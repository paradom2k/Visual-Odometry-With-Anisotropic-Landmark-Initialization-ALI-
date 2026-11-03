#include "data/bag_reader.h"

#include "common/log.h"

#include <algorithm>
#include <cstring>
#include <windows.h>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace jo {
namespace {

constexpr std::size_t kMaxHeaderBytes = 1u << 20;  // 1 MiB sanity bound
constexpr char kBagMagic[] = "#ROSBAG V2.0\n";
constexpr std::size_t kBagMagicLength = 13;

/// One parsed record header: a sequence of "key=value" fields.
using Fields = std::map<std::string, std::string>;

bool parseHeaderFields(const std::uint8_t* data, std::size_t size,
                       Fields* out) {
  std::size_t offset = 0;
  while (offset + 4 <= size) {
    std::uint32_t length = 0;
    std::memcpy(&length, data + offset, sizeof(length));
    offset += 4;
    if (offset + length > size) {
      return false;
    }
    const std::string key(reinterpret_cast<const char*>(data + offset), length);
    offset += length;
    const std::size_t equals = key.find('=');
    if (equals != std::string::npos && equals > 0) {
      (*out)[key.substr(0, equals)] = key.substr(equals + 1);
    }
  }
  return true;
}

std::uint8_t opByte(const Fields& fields) {
  auto it = fields.find("op");
  if (it == fields.end() || it->second.empty()) {
    return 0;
  }
  return static_cast<std::uint8_t>(it->second[0]);
}

std::uint32_t readU32(const std::string& raw) {
  if (raw.size() < 4) {
    return 0;
  }
  std::uint32_t value = 0;
  std::memcpy(&value, raw.data(), sizeof(value));
  return value;
}

bool readLengthPrefixed(const std::uint8_t* data, std::size_t size,
                        std::size_t* offset, std::string* out) {
  if (*offset + 4 > size) {
    return false;
  }
  std::uint32_t length = 0;
  std::memcpy(&length, data + *offset, sizeof(length));
  *offset += 4;
  if (*offset + length > size) {
    return false;
  }
  out->assign(reinterpret_cast<const char*>(data + *offset), length);
  *offset += length;
  return true;
}

/// Heuristics that identify the camera stream from a connection topic name.
///
/// The dataset is inconsistent: MINI names its image connections
/// "/camNN/image_raw" while the message-level topic is
/// "/camNN/image_raw/compressed"; MATE names them "/camNN/image_raw/compressed"
/// directly.  IMU and point clouds must never be mistaken for an image stream.
bool looksLikeImageTopic(const std::string& topic) {
  if (topic.empty()) {
    return false;
  }
  const bool has_cam = topic.find("cam") != std::string::npos ||
                       topic.find("image") != std::string::npos;
  if (!has_cam) {
    return false;
  }
  if (topic.find("imu") != std::string::npos ||
      topic.find("points") != std::string::npos ||
      topic.find("scan") != std::string::npos) {
    return false;
  }
  return true;
}

struct ChunkVisitor {
  std::map<std::uint32_t, BagConnection>* connections = nullptr;
  std::map<std::uint32_t, std::size_t>* counts = nullptr;
  std::uint64_t* chunks = nullptr;
  std::string* chunk_error = nullptr;

  // Image extraction state.
  bool want_images = false;
  std::string topic_filter;
  bool image_connection_known = false;
  std::uint32_t image_connection = 0;
  /// Authoritative topic name taken from the message-level header, which is
  /// what the connection name usually fails to match.
  std::string resolved_image_topic;
  std::size_t max_frames = 0;
  /// Index of the first frame the caller wants; earlier messages are counted and
  /// dropped.
  std::size_t first_frame = 0;
  std::size_t skipped = 0;
  std::vector<ImageFrame>* frames = nullptr;
  /// SUPPLIED -- raw-payload mode.  When `on_raw` is set, a message that has passed the topic
  /// filter is handed over as (stamp, bytes) and is NOT decoded as an image.  This exists so that
  /// tools/imu_dump.cpp can read sensor_msgs/Imu out of a bag with the walk that is already proven
  /// to work on the 4Seasons recording: the same walk reads that bag's images today
  /// (9,487 of them, see bin/jo_frames_dump.exe), and the topic filter above already matches by
  /// exact topic name when one is given, so nothing about the walk had to change.
  ///
  /// Behaviour is unchanged unless a caller sets this: the branch that uses it is guarded by
  /// `on_raw != nullptr`, so the recovered image path is untouched.
  std::function<bool(double stamp, const std::uint8_t* payload, std::size_t size)>* on_raw = nullptr;
  std::size_t raw_delivered = 0;
  bool stop = false;
};

void visitInnerRecords(const std::uint8_t* data, std::size_t size,
                       ChunkVisitor* visitor) {
  std::size_t offset = 0;
  while (offset + 8 <= size && !visitor->stop) {
    std::uint32_t header_length = 0;
    std::memcpy(&header_length, data + offset, sizeof(header_length));
    offset += 4;
    if (header_length == 0 || header_length > kMaxHeaderBytes ||
        offset + header_length + 4 > size) {
      return;
    }
    Fields fields;
    if (!parseHeaderFields(data + offset, header_length, &fields)) {
      return;
    }
    offset += header_length;

    std::uint32_t data_length = 0;
    std::memcpy(&data_length, data + offset, sizeof(data_length));
    offset += 4;
    if (offset + data_length > size) {
      return;
    }
    const std::uint8_t* payload = data + offset;
    offset += data_length;

    const std::uint8_t op = opByte(fields);
    if (op == 0x07) {  // connection, repeated inside every chunk
      BagConnection connection;
      connection.id = readU32(fields["conn"]);
      auto topic = fields.find("topic");
      if (topic != fields.end()) {
        connection.topic = topic->second;
      }
      auto type = fields.find("type");
      if (type != fields.end()) {
        connection.type = type->second;
      }
      (*visitor->connections)[connection.id] = connection;
      continue;
    }
    if (op != 0x02) {  // not a message
      continue;
    }

    const std::uint32_t conn = readU32(fields["conn"]);
    (*visitor->counts)[conn] += 1;

    // GIO_GVIL FIX.  This guard reads as "the walk only decodes when the caller wants images", and it is
    // placed BEFORE the raw-payload branch below -- so a caller that asked for raw messages and no images
    // had every message dropped here and the walk reported "no messages on topic", which names the topic
    // and not the guard.  `readRaw` never sets `want_images`, so the raw outlet this file documents as the
    // one that works on the 4Seasons recording delivered nothing at all.  The condition now stands aside
    // when a raw callback is registered; the image path is unchanged.
    if (!visitor->want_images && visitor->on_raw == nullptr) {
      continue;
    }

    // ROS1 message records carry only `conn`, `op` and `time`; the topic lives
    // on the connection record.  Resolve it through the connection map, and
    // prefer the connection record's name if a message ever did carry one.
    std::string message_topic;
    auto topic_field = fields.find("topic");
    if (topic_field != fields.end()) {
      message_topic = topic_field->second;
    } else {
      auto conn_it = visitor->connections->find(conn);
      if (conn_it != visitor->connections->end()) {
        message_topic = conn_it->second.topic;
      }
    }

    // SUPPLIED -- raw-payload mode, and it must sit BEFORE the connection latch below.  That latch
    // is the recovered image path's design: the first message whose topic matches pins
    // `image_connection`, and every later message from another connection is dropped (the
    // `conn != visitor->image_connection` test further down).  Correct for images, fatal here -- on
    // this bag the camera connection is seen first, so /imu_raw was dropped and the walk reported
    // "no messages on topic '/imu_raw'".  Raw mode therefore matches the topic itself, per message,
    // and never latches.
    if (visitor->on_raw != nullptr) {
      if (visitor->topic_filter.empty() || message_topic != visitor->topic_filter) {
        continue;
      }
      if (data_length < 12) {
        continue;
      }
      std::uint32_t raw_secs = 0, raw_nsecs = 0;
      std::memcpy(&raw_secs, payload + 4, sizeof(raw_secs));
      std::memcpy(&raw_nsecs, payload + 8, sizeof(raw_nsecs));
      const double raw_stamp = static_cast<double>(raw_secs) + static_cast<double>(raw_nsecs) * 1e-9;
      ++visitor->raw_delivered;
      const std::size_t raw_limit = visitor->max_frames;
      if (!(*visitor->on_raw)(raw_stamp, payload, data_length)) {
        visitor->stop = true;
        continue;
      }
      if (raw_limit != 0 && visitor->raw_delivered >= raw_limit) {
        visitor->stop = true;
      }
      continue;
    }

    if (!visitor->image_connection_known) {
      const bool candidate =
          visitor->topic_filter.empty()
              ? looksLikeImageTopic(message_topic)
              : (message_topic == visitor->topic_filter);
      if (!candidate) {
        continue;
      }
      visitor->image_connection = conn;
      visitor->image_connection_known = true;
      visitor->resolved_image_topic = message_topic;
      std::string declared_type;
      {
        auto type_entry = visitor->connections->find(conn);
        if (type_entry != visitor->connections->end()) {
          declared_type = type_entry->second.type;
        }
      }
      logInfoF("bag: image topic '%s' (connection %u, declared type '%s')",
               message_topic.c_str(), conn, declared_type.c_str());
    }
    if (conn != visitor->image_connection) {
      continue;
    }

    // SUPPLIED -- raw-payload mode: the message has passed the topic filter, so hand its bytes over
    // and skip the image decode below entirely.  The stamp is read from the payload's own
    // std_msgs/Header (uint32 seq, then seconds, then nanoseconds), the same layout the image path
    // reads at its own cursor -- so the two agree on time to the nanosecond.
    if (visitor->on_raw != nullptr) {
      std::uint32_t raw_secs = 0, raw_nsecs = 0;
      if (data_length < 12) {
        continue;
      }
      std::memcpy(&raw_secs, payload + 4, sizeof(raw_secs));
      std::memcpy(&raw_nsecs, payload + 8, sizeof(raw_nsecs));
      const double raw_stamp = static_cast<double>(raw_secs) + static_cast<double>(raw_nsecs) * 1e-9;
      ++visitor->raw_delivered;
      const std::size_t raw_limit = visitor->max_frames;
      if (!(*visitor->on_raw)(raw_stamp, payload, data_length)) {
        visitor->stop = true;
        continue;
      }
      if (raw_limit != 0 && visitor->raw_delivered >= raw_limit) {
        visitor->stop = true;
      }
      continue;
    }

    // Random access: the messages before the wanted index are skipped *before*
    // their payload is copied.  Copying them would work too and is what this used
    // to do, but a step through a six-hundred-frame sequence then re-reads sixty
    // megabytes for every step.
    if (visitor->skipped < visitor->first_frame) {
      ++visitor->skipped;
      continue;
    }

    // sensor_msgs/CompressedImage layout:
    //   Header header  -> uint32 seq, uint32 secs, uint32 nsecs, string frame_id
    //   string format
    //   uint8[] data
    //
    // sensor_msgs/Image layout (what the 4Seasons recording publishes on /camera/image_raw):
    //   Header header  -> uint32 seq, uint32 secs, uint32 nsecs, string frame_id
    //   uint32 height, uint32 width, string encoding, uint8 is_bigendian, uint32 step,
    //   uint8[] data
    //
    // Which one it is comes from the connection's declared type, not from the payload: a raw raster
    // whose first bytes happen to look like a length prefix would otherwise parse as a plausible
    // frame with garbage pixels, which is the kind of bug that survives to a trajectory.
    const BagConnection* connection = nullptr;
    {
      auto conn_entry = visitor->connections->find(conn);
      if (conn_entry != visitor->connections->end()) {
        connection = &conn_entry->second;
      }
    }
    const bool raw_image = [&]() {
      const std::string declared =
          connection != nullptr ? connection->type : std::string();
      const bool type_compressed = declared.find("CompressedImage") != std::string::npos;
      const bool type_image = declared.find("Image") != std::string::npos;
      // The 4Seasons bag leaves the connection type empty (it is published by a converter that does
      // not fill it in), so there the topic name decides: every compressed camera connection this
      // project has met names itself `.../compressed` (the G-drive bags), and a bare
      // `.../image_raw` is a raw `sensor_msgs/Image`.
      const bool name_compressed = message_topic.find("compressed") != std::string::npos;
      if (type_compressed) {
        return false;
      }
      if (type_image) {
        return true;
      }
      return declared.empty() && !name_compressed;
    }();
    std::size_t cursor = 0;
    if (data_length < 12) {
      continue;
    }
    ImageFrame frame;
    std::memcpy(&frame.sequence, payload + cursor, sizeof(std::uint32_t));
    cursor += 4;
    std::uint32_t secs = 0;
    std::uint32_t nsecs = 0;
    std::memcpy(&secs, payload + cursor, sizeof(secs));
    cursor += 4;
    std::memcpy(&nsecs, payload + cursor, sizeof(nsecs));
    cursor += 4;
    frame.time = static_cast<double>(secs) + static_cast<double>(nsecs) * 1e-9;
    if (!readLengthPrefixed(payload, data_length, &cursor, &frame.frame_id)) {
      continue;
    }
    if (raw_image) {
      std::uint32_t height = 0;
      std::uint32_t width = 0;
      std::uint32_t step = 0;
      if (cursor + 8 > data_length) {
        logWarnF("bag: raw image message too short (%zu of %zu)", cursor, data_length);
        continue;
      }
      std::memcpy(&height, payload + cursor, 4);
      cursor += 4;
      std::memcpy(&width, payload + cursor, 4);
      cursor += 4;
      if (!readLengthPrefixed(payload, data_length, &cursor, &frame.encoding)) {
        continue;
      }
      if (cursor + 1 > data_length) {
        continue;
      }
      ++cursor;  // is_bigendian
      if (cursor + 4 > data_length) {
        continue;
      }
      std::memcpy(&step, payload + cursor, 4);
      cursor += 4;
      std::uint32_t blob_length = 0;
      if (cursor + 4 > data_length) {
        continue;
      }
      std::memcpy(&blob_length, payload + cursor, 4);
      cursor += 4;
      if (blob_length == 0 || cursor + blob_length > data_length) {
        logWarnF("bag: raw image %ux%u step %u needs %u byte(s) at %zu of %zu",
                 width, height, step, blob_length, cursor, data_length);
        continue;
      }
      if (visitor->frames->empty()) {
        logInfoF("bag: raw image %ux%u encoding '%s' step %u (%u byte(s))",
                 width, height, frame.encoding.c_str(), step, blob_length);
      }
      frame.width = static_cast<int>(width);
      frame.height = static_cast<int>(height);
      frame.step = static_cast<int>(step);
      frame.raw_raster = true;
      frame.payload.assign(payload + cursor, payload + cursor + blob_length);
      visitor->frames->push_back(std::move(frame));
      if (visitor->max_frames > 0 &&
          visitor->frames->size() >= visitor->max_frames) {
        visitor->stop = true;
      }
      continue;
    }

    if (!readLengthPrefixed(payload, data_length, &cursor, &frame.encoding)) {
      continue;
    }
    if (cursor + 4 > data_length) {
      continue;
    }
    std::uint32_t blob_length = 0;
    std::memcpy(&blob_length, payload + cursor, sizeof(blob_length));
    cursor += 4;
    if (cursor + blob_length > data_length) {
      continue;
    }

    frame.payload.assign(payload + cursor, payload + cursor + blob_length);
    int width = 0;
    int height = 0;
    int components = 0;
    if (jpegDimensions(frame.payload.data(), frame.payload.size(), &width,
                       &height, &components)) {
      frame.width = width;
      frame.height = height;
    }
    visitor->frames->push_back(std::move(frame));

    if (visitor->max_frames > 0 &&
        visitor->frames->size() >= visitor->max_frames) {
      visitor->stop = true;
    }
  }
}

bool walkBag(const std::uint8_t* data, std::size_t size, ChunkVisitor* visitor,
             std::string* error) {
  if (size < kBagMagicLength ||
      std::memcmp(data, kBagMagic, kBagMagicLength) != 0) {
    if (error != nullptr) {
      *error = "not a ROS1 bag v2 file";
    }
    return false;
  }

  std::size_t offset = kBagMagicLength;
  while (offset + 8 <= size && !visitor->stop) {
    std::uint32_t header_length = 0;
    std::memcpy(&header_length, data + offset, sizeof(header_length));
    offset += 4;
    if (header_length == 0 || header_length > kMaxHeaderBytes ||
        offset + header_length + 4 > size) {
      break;
    }
    Fields fields;
    if (!parseHeaderFields(data + offset, header_length, &fields)) {
      break;
    }
    offset += header_length;

    std::uint32_t data_length = 0;
    std::memcpy(&data_length, data + offset, sizeof(data_length));
    offset += 4;
    if (offset + data_length > size) {
      break;
    }
    const std::uint8_t* payload = data + offset;
    offset += data_length;

    // SUPPLIED -- TOP-LEVEL message records.  visitInnerRecords() handles messages that live inside
    // a chunk; nothing here ever looked at a message record that sits at the top level, and the
    // 4Seasons recording carries messages both ways.  That asymmetry is what the probes showed:
    // /camera/image_raw comes out of a chunk and reads fine, while /imu_raw -- connection 0, in the
    // file header -- delivered nothing, and its 626,881 messages could not be explained by
    // compression (measured: chunks are compression=none, no bz2 or lz4 in the file at all).
    // Raw mode only: the image path is untouched, because the branch is guarded on on_raw.
    if (visitor->on_raw != nullptr && opByte(fields) == 0x02) {
      auto conn_field = fields.find("conn");
      if (conn_field == fields.end()) {
        continue;
      }
      auto connection = visitor->connections->find(readU32(conn_field->second));
      if (connection == visitor->connections->end()) {
        continue;
      }
      if (visitor->topic_filter.empty() || connection->second.topic != visitor->topic_filter) {
        continue;
      }
      if (data_length < 12) {
        continue;
      }
      std::uint32_t top_secs = 0, top_nsecs = 0;
      std::memcpy(&top_secs, payload + 4, sizeof(top_secs));
      std::memcpy(&top_nsecs, payload + 8, sizeof(top_nsecs));
      const double top_stamp = static_cast<double>(top_secs) + static_cast<double>(top_nsecs) * 1e-9;
      ++visitor->raw_delivered;
      const std::size_t top_limit = visitor->max_frames;
      if (!(*visitor->on_raw)(top_stamp, payload, data_length)) {
        visitor->stop = true;
        continue;
      }
      if (top_limit != 0 && visitor->raw_delivered >= top_limit) {
        visitor->stop = true;
      }
      continue;
    }

    const std::uint8_t op = opByte(fields);
    if (op == 0x07) {
      BagConnection connection;
      connection.id = readU32(fields["conn"]);
      auto topic = fields.find("topic");
      if (topic != fields.end()) {
        connection.topic = topic->second;
      }
      auto type = fields.find("type");
      if (type != fields.end()) {
        connection.type = type->second;
      }
      (*visitor->connections)[connection.id] = connection;
    } else if (op == 0x05) {
      (*visitor->chunks) += 1;
      auto compression = fields.find("compression");
      if (compression != fields.end() && compression->second != "none") {
        if (visitor->chunk_error != nullptr && visitor->chunk_error->empty()) {
          *visitor->chunk_error =
              "chunk compression '" + compression->second +
              "' is not supported by this reader";
        }
        continue;
      }
      visitInnerRecords(payload, data_length, visitor);
    }
  }
  return true;
}

bool loadWholeFile(const std::string& path, std::vector<std::uint8_t>* data,
                   std::string* error) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    if (error != nullptr) {
      *error = "cannot open bag: " + path;
    }
    return false;
  }
  stream.seekg(0, std::ios::end);
  const std::streamoff size = stream.tellg();
  stream.seekg(0, std::ios::beg);
  if (size <= 0) {
    if (error != nullptr) {
      *error = "empty bag: " + path;
    }
    return false;
  }
  data->resize(static_cast<std::size_t>(size));
  stream.read(reinterpret_cast<char*>(data->data()),
              static_cast<std::streamsize>(data->size()));
  return true;
}

}  // namespace

bool indexBagImages(const std::string& path, const std::string& topic,
                    BagImageIndex* out, std::string* error) {
  if (out == nullptr) {
    return false;
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    if (error != nullptr) {
      *error = "cannot open bag: " + path;
    }
    return false;
  }
  char magic[kBagMagicLength];
  in.read(magic, static_cast<std::streamsize>(kBagMagicLength));
  if (in.gcount() != static_cast<std::streamsize>(kBagMagicLength) ||
      std::memcmp(magic, kBagMagic, kBagMagicLength) != 0) {
    if (error != nullptr) {
      *error = "not a ROS1 bag v2 file: " + path;
    }
    return false;
  }

  std::map<std::uint32_t, std::string> topics;
  std::map<std::uint32_t, std::vector<BagImageLocation>> located;
  std::uint64_t position = kBagMagicLength;
  std::uint64_t chunk_data_offset = 0;
  bool have_chunk = false;
  bool chunk_compressed = false;
  bool skipped_compressed = false;
  std::size_t index_records = 0;

  for (;;) {
    std::uint32_t header_length = 0;
    in.read(reinterpret_cast<char*>(&header_length), 4);
    if (in.gcount() != 4 || header_length == 0 ||
        header_length > kMaxHeaderBytes) {
      break;
    }
    std::vector<std::uint8_t> header(header_length);
    in.read(reinterpret_cast<char*>(header.data()),
            static_cast<std::streamsize>(header_length));
    if (in.gcount() != static_cast<std::streamsize>(header_length)) {
      break;
    }
    Fields fields;
    if (!parseHeaderFields(header.data(), header_length, &fields)) {
      break;
    }
    std::uint32_t data_length = 0;
    in.read(reinterpret_cast<char*>(&data_length), 4);
    if (in.gcount() != 4) {
      break;
    }
    const std::uint8_t op = opByte(fields);
    const std::uint64_t record_size = 4ull + header_length + 4ull + data_length;

    if (op == 0x07) {  // connection record: the only place a topic is named
      auto conn = fields.find("conn");
      auto name = fields.find("topic");
      if (conn != fields.end() && name != fields.end()) {
        topics[readU32(conn->second)] = name->second;
      }
      in.seekg(static_cast<std::streamoff>(data_length), std::ios::cur);
    } else if (op == 0x05) {  // chunk: its payload is skipped, never read
      auto compression = fields.find("compression");
      chunk_compressed =
          !(compression != fields.end() && compression->second == "none");
      chunk_data_offset = position + 4ull + header_length + 4ull;
      have_chunk = true;
      if (chunk_compressed) {
        skipped_compressed = true;
      }
      in.seekg(static_cast<std::streamoff>(data_length), std::ios::cur);
    } else if (op == 0x04) {  // index of the chunk that precedes it
      ++index_records;
      std::vector<std::uint8_t> data(data_length);
      in.read(reinterpret_cast<char*>(data.data()),
              static_cast<std::streamsize>(data_length));
      if (in.gcount() != static_cast<std::streamsize>(data_length)) {
        break;
      }
      auto conn = fields.find("conn");
      auto count_field = fields.find("count");
      if (have_chunk && !chunk_compressed && conn != fields.end() &&
          count_field != fields.end()) {
        const std::uint32_t connection = readU32(conn->second);
        const std::uint32_t count = readU32(count_field->second);
        std::vector<BagImageLocation>& list = located[connection];
        list.reserve(list.size() + count);
        for (std::uint32_t i = 0; i < count; ++i) {
          const std::size_t offset = static_cast<std::size_t>(i) * 12u;
          if (offset + 12u > data.size()) {
            break;
          }
          std::uint32_t secs = 0;
          std::uint32_t nsecs = 0;
          std::uint32_t in_chunk = 0;
          std::memcpy(&secs, data.data() + offset, 4);
          std::memcpy(&nsecs, data.data() + offset + 4, 4);
          std::memcpy(&in_chunk, data.data() + offset + 8, 4);
          BagImageLocation location;
          location.time =
              static_cast<double>(secs) + static_cast<double>(nsecs) * 1e-9;
          location.chunk_data_offset = chunk_data_offset;
          location.offset_in_chunk = in_chunk;
          list.push_back(location);
        }
      }
    } else {
      in.seekg(static_cast<std::streamoff>(data_length), std::ios::cur);
    }
    position += record_size;
  }

  if (skipped_compressed) {
    logWarnF("bag: %s : some chunks are compressed; their images are not indexed",
             path.c_str());
  }

  // Resolve the image connection.  Done after the walk because the connection
  // records may be spread over the file, and an explicit topic is matched
  // exactly while an empty one takes the first image-like topic.
  std::uint32_t image_connection = 0;
  bool found = false;
  for (const auto& entry : topics) {
    auto list = located.find(entry.first);
    if (list == located.end() || list->second.empty()) {
      continue;
    }
    const bool candidate = topic.empty() ? looksLikeImageTopic(entry.second)
                                         : entry.second == topic;
    if (!candidate) {
      continue;
    }
    image_connection = entry.first;
    found = true;
    break;
  }
  if (!found) {
    if (error != nullptr) {
      *error = topic.empty() ? ("no image topic in " + path)
                             : ("no images on topic '" + topic + "' in " + path);
    }
    return false;
  }

  out->path = path;
  out->topic = topics[image_connection];
  out->connection = image_connection;
  out->readable = true;
  out->entries = std::move(located[image_connection]);
  std::sort(out->entries.begin(), out->entries.end(),
            [](const BagImageLocation& a, const BagImageLocation& b) {
              return a.time < b.time;
            });
  if (out->entries.empty()) {
    if (error != nullptr) {
      *error = "no indexed images on topic '" + out->topic + "' in " + path;
    }
    return false;
  }
  logInfoF("bag: %s : '%s', %zu image(s), %.3f .. %.3f (%zu index record(s))",
           path.c_str(), out->topic.c_str(), out->entries.size(),
           out->entries.front().time, out->entries.back().time, index_records);
  return true;
}

bool indexBagImagesCached(const std::string& path, const std::string& topic,
                          const std::string& cache_dir, BagImageIndex* out,
                          std::string* error) {
  // Win32 rather than <filesystem>, because this tree is built as C++14: that is the standard upstream
  // ORB-SLAM3 is written to and one binary cannot mix standards across a library boundary without risking
  // the standard-library ABI.  The calls below do exactly what the three <filesystem> calls did -- make the
  // directory, size the bag, name the cache file -- and nothing else about the function changes.
  if (cache_dir.empty()) {
    return indexBagImages(path, topic, out, error);
  }
  CreateDirectoryA(cache_dir.c_str(), nullptr);
  WIN32_FILE_ATTRIBUTE_DATA bag_attributes;
  if (GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &bag_attributes) == 0) {
    if (error != nullptr) {
      *error = "cannot size " + path;
    }
    return false;
  }
  const unsigned long long bytes =
      (static_cast<unsigned long long>(bag_attributes.nFileSizeHigh) << 32) |
      static_cast<unsigned long long>(bag_attributes.nFileSizeLow);
  std::string stem = path;
  const std::size_t last_slash = stem.find_last_of("/\\");
  if (last_slash != std::string::npos) {
    stem = stem.substr(last_slash + 1);
  }
  const std::size_t last_dot = stem.find_last_of('.');
  if (last_dot != std::string::npos) {
    stem = stem.substr(0, last_dot);
  }
  const std::string cache_file = cache_dir + "/" + stem + ".idx";
  const bool cache_exists =
      GetFileAttributesA(cache_file.c_str()) != INVALID_FILE_ATTRIBUTES;
  // A cache that names a different size is a cache for a different bag: re-recorded bags
  // keep their name, and reading one of those with the old offsets would decode garbage.
  if (cache_exists) {
    std::ifstream in(cache_file);
    std::string header;
    if (in && std::getline(in, header)) {
      const std::size_t comma = header.find(',');
      if (comma != std::string::npos &&
          header.substr(0, comma) == std::to_string(bytes)) {
        BagImageIndex loaded;
        loaded.path = path;
        loaded.readable = true;
        std::string line;
        while (std::getline(in, line)) {
          if (line.empty()) {
            continue;
          }
          // time,chunk_data_offset,offset_in_chunk
          const std::size_t first = line.find(',');
          const std::size_t second =
              first == std::string::npos ? std::string::npos : line.find(',', first + 1);
          if (second == std::string::npos) {
            continue;
          }
          try {
            BagImageLocation entry;
            entry.time = std::stod(line.substr(0, first));
            entry.chunk_data_offset =
                static_cast<std::uint64_t>(std::stoull(line.substr(first + 1, second - first - 1)));
            entry.offset_in_chunk =
                static_cast<std::uint32_t>(std::stoul(line.substr(second + 1)));
            loaded.entries.push_back(entry);
          } catch (const std::exception&) {
            loaded.entries.clear();
            break;
          }
        }
        if (!loaded.entries.empty()) {
          loaded.topic = topic;
          *out = std::move(loaded);
          return true;
        }
      }
    }
  }
  if (!indexBagImages(path, topic, out, error)) {
    return false;
  }
  std::ofstream cache_out(cache_file);
  if (cache_out) {
    cache_out.precision(17);
    cache_out << bytes << ',' << out->entries.size() << '\n';
    for (const BagImageLocation& entry : out->entries) {
      cache_out << entry.time << ',' << entry.chunk_data_offset << ','
                << entry.offset_in_chunk << '\n';
    }
  }
  return true;
}

bool readBagImage(const std::string& path, const BagImageIndex& index,
                  std::size_t frame, ImageFrame* out, std::string* error) {
  if (out == nullptr || frame >= index.entries.size()) {
    if (error != nullptr) {
      *error = "image index out of range";
    }
    return false;
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    if (error != nullptr) {
      *error = "cannot open bag: " + path;
    }
    return false;
  }
  const BagImageLocation& location = index.entries[frame];
  in.seekg(static_cast<std::streamoff>(location.chunk_data_offset +
                                       location.offset_in_chunk),
           std::ios::beg);

  std::uint32_t header_length = 0;
  in.read(reinterpret_cast<char*>(&header_length), 4);
  if (in.gcount() != 4 || header_length == 0 ||
      header_length > kMaxHeaderBytes) {
    if (error != nullptr) {
      *error = "bad message header at the indexed offset";
    }
    return false;
  }
  std::vector<std::uint8_t> header(header_length);
  in.read(reinterpret_cast<char*>(header.data()),
          static_cast<std::streamsize>(header_length));
  if (in.gcount() != static_cast<std::streamsize>(header_length)) {
    if (error != nullptr) {
      *error = "truncated message header";
    }
    return false;
  }
  std::uint32_t data_length = 0;
  in.read(reinterpret_cast<char*>(&data_length), 4);
  if (in.gcount() != 4) {
    if (error != nullptr) {
      *error = "truncated message length";
    }
    return false;
  }
  std::vector<std::uint8_t> payload(data_length);
  in.read(reinterpret_cast<char*>(payload.data()),
          static_cast<std::streamsize>(data_length));
  if (in.gcount() != static_cast<std::streamsize>(data_length)) {
    if (error != nullptr) {
      *error = "truncated message payload";
    }
    return false;
  }

  // sensor_msgs/CompressedImage: header (seq, stamp, frame_id), format, data.
  // sensor_msgs/Image:           header (seq, stamp, frame_id), height, width, encoding,
  //                              is_bigendian, step, data.
  // Which one this connection carries follows the same rule the batch reader uses: a
  // `.../compressed` topic is compressed, anything else is raw (the 4Seasons bag leaves its
  // connection type empty, so the topic name is what is left to go on).
  const bool raw_image = index.topic.find("compressed") == std::string::npos;
  std::size_t cursor = 0;
  if (data_length < 12) {
    if (error != nullptr) {
      *error = "message too short for an image header";
    }
    return false;
  }
  ImageFrame image;
  std::memcpy(&image.sequence, payload.data() + cursor, sizeof(std::uint32_t));
  cursor += 4;
  std::uint32_t secs = 0;
  std::uint32_t nsecs = 0;
  std::memcpy(&secs, payload.data() + cursor, sizeof(secs));
  cursor += 4;
  std::memcpy(&nsecs, payload.data() + cursor, sizeof(nsecs));
  cursor += 4;
  image.time = static_cast<double>(secs) + static_cast<double>(nsecs) * 1e-9;
  if (!readLengthPrefixed(payload.data(), data_length, &cursor,
                          &image.frame_id)) {
    if (error != nullptr) {
      *error = "malformed image header";
    }
    return false;
  }

  if (raw_image) {
    std::uint32_t height = 0;
    std::uint32_t width = 0;
    std::uint32_t step = 0;
    if (cursor + 8 > data_length) {
      if (error != nullptr) *error = "truncated raw image header";
      return false;
    }
    std::memcpy(&height, payload.data() + cursor, 4);
    cursor += 4;
    std::memcpy(&width, payload.data() + cursor, 4);
    cursor += 4;
    if (!readLengthPrefixed(payload.data(), data_length, &cursor,
                            &image.encoding)) {
      if (error != nullptr) *error = "malformed raw image encoding";
      return false;
    }
    if (cursor + 5 > data_length) {
      if (error != nullptr) *error = "truncated raw image header";
      return false;
    }
    ++cursor;  // is_bigendian
    std::memcpy(&step, payload.data() + cursor, 4);
    cursor += 4;
    std::uint32_t blob_length = 0;
    std::memcpy(&blob_length, payload.data() + cursor, 4);
    cursor += 4;
    if (blob_length == 0 || cursor + blob_length > data_length) {
      if (error != nullptr) *error = "raw image blob runs past the message";
      return false;
    }
    image.width = static_cast<int>(width);
    image.height = static_cast<int>(height);
    image.step = static_cast<int>(step);
    image.raw_raster = true;
    image.payload.assign(
        payload.begin() + static_cast<std::ptrdiff_t>(cursor),
        payload.begin() + static_cast<std::ptrdiff_t>(cursor + blob_length));
    *out = std::move(image);
    return true;
  }

  if (!readLengthPrefixed(payload.data(), data_length, &cursor,
                          &image.encoding)) {
    if (error != nullptr) {
      *error = "malformed image header";
    }
    return false;
  }
  if (cursor + 4 > data_length) {
    if (error != nullptr) {
      *error = "missing image blob length";
    }
    return false;
  }
  std::uint32_t blob_length = 0;
  std::memcpy(&blob_length, payload.data() + cursor, sizeof(blob_length));
  cursor += 4;
  if (cursor + blob_length > data_length) {
    if (error != nullptr) {
      *error = "image blob runs past the message";
    }
    return false;
  }
  image.payload.assign(payload.begin() + static_cast<std::ptrdiff_t>(cursor),
                       payload.begin() +
                           static_cast<std::ptrdiff_t>(cursor + blob_length));
  int width = 0;
  int height = 0;
  int components = 0;
  if (jpegDimensions(image.payload.data(), image.payload.size(), &width, &height,
                     &components)) {
    image.width = width;
    image.height = height;
  }
  *out = std::move(image);
  return true;
}

bool jpegDimensions(const std::uint8_t* data, std::size_t size, int* width,
                    int* height, int* components) {
  if (data == nullptr || size < 4 || data[0] != 0xFF || data[1] != 0xD8) {
    return false;
  }
  std::size_t offset = 2;
  while (offset + 9 < size) {
    if (data[offset] != 0xFF) {
      ++offset;
      continue;
    }
    const std::uint8_t marker = data[offset + 1];
    if (marker == 0xD8 || marker == 0xD9 || (marker >= 0xD0 && marker <= 0xD7) ||
        marker == 0x01) {
      offset += 2;
      continue;
    }
    if (offset + 4 > size) {
      return false;
    }
    const std::size_t segment_length =
        (static_cast<std::size_t>(data[offset + 2]) << 8) | data[offset + 3];
    const bool is_sof = (marker >= 0xC0 && marker <= 0xC3) ||
                        (marker >= 0xC5 && marker <= 0xC7) ||
                        (marker >= 0xC9 && marker <= 0xCB) ||
                        (marker >= 0xCD && marker <= 0xCF);
    if (is_sof) {
      if (offset + 9 > size) {
        return false;
      }
      *height = (static_cast<int>(data[offset + 5]) << 8) | data[offset + 6];
      *width = (static_cast<int>(data[offset + 7]) << 8) | data[offset + 8];
      *components = data[offset + 9];
      return *width > 0 && *height > 0;
    }
    offset += 2 + segment_length;
  }
  return false;
}

bool BagReader::inspect(const std::string& path, BagInfo* out,
                        std::string* error) {
  std::vector<std::uint8_t> data;
  if (!loadWholeFile(path, &data, error)) {
    return false;
  }

  BagInfo info;
  info.path = path;
  info.file_size = static_cast<std::uint64_t>(data.size());

  std::map<std::uint32_t, BagConnection> connections;
  std::map<std::uint32_t, std::size_t> counts;
  std::string chunk_error;

  ChunkVisitor visitor;
  visitor.connections = &connections;
  visitor.counts = &counts;
  visitor.chunks = &info.chunk_count;
  visitor.chunk_error = &chunk_error;

  if (!walkBag(data.data(), data.size(), &visitor, error)) {
    return false;
  }

  for (const auto& entry : connections) {
    info.connections.push_back(entry.second);
  }
  for (const auto& entry : counts) {
    info.message_counts.emplace_back(entry.first, entry.second);
  }
  info.valid = true;
  if (!chunk_error.empty()) {
    logWarnF("bag: %s : %s", path.c_str(), chunk_error.c_str());
  }
  *out = std::move(info);
  return true;
}

bool BagReader::readImages(const std::string& path, const std::string& topic,
                           std::size_t max_frames,
                           std::vector<ImageFrame>* out, std::string* error) {
  return readImages(path, topic, 0, max_frames, out, error);
}

bool BagReader::readImages(const std::string& path, const std::string& topic,
                           std::size_t first_frame, std::size_t count,
                           std::vector<ImageFrame>* out, std::string* error) {
  std::vector<std::uint8_t> data;
  if (!loadWholeFile(path, &data, error)) {
    return false;
  }

  std::map<std::uint32_t, BagConnection> connections;
  std::map<std::uint32_t, std::size_t> counts;
  std::uint64_t chunks = 0;
  std::string chunk_error;
  std::vector<ImageFrame> frames;

  ChunkVisitor visitor;
  visitor.connections = &connections;
  visitor.counts = &counts;
  visitor.chunks = &chunks;
  visitor.chunk_error = &chunk_error;
  visitor.want_images = true;
  visitor.topic_filter = topic;
  visitor.first_frame = first_frame;
  visitor.max_frames = count;
  visitor.frames = &frames;

  if (!walkBag(data.data(), data.size(), &visitor, error)) {
    return false;
  }
  if (frames.empty()) {
    // Report what was actually present; a bare "not found" is not actionable.
    std::ostringstream detail;
    detail << "connections=" << connections.size() << " [";
    for (const auto& entry : connections) {
      detail << entry.first << ":" << entry.second.topic << " ";
    }
    detail << "] messages=" << counts.size() << " [";
    for (const auto& entry : counts) {
      detail << entry.first << ":" << entry.second << " ";
    }
    detail << "] image_connection_known=" << visitor.image_connection_known;
    logWarnF("bag: %s : %s", path.c_str(), detail.str().c_str());
    if (error != nullptr) {
      if (!chunk_error.empty()) {
        *error = chunk_error;
      } else if (topic.empty()) {
        *error = "no compressed-image topic found in " + path;
      } else {
        *error = "no messages on topic '" + topic + "' in " + path;
      }
    }
    return false;
  }

  *out = std::move(frames);
  return true;
}

// SUPPLIED -- the raw-payload sibling of readImages().  Same file, same walk, same topic filter;
// the only difference is that matching messages are handed over as bytes instead of being decoded
// as images.  tools/imu_dump.cpp needs exactly this to read sensor_msgs/Imu out of the 4Seasons
// recording: that bag's images come out of this walk today (9,487 of them), while the PointCloud2
// walker in src\lidar\lidar_bag.h delivers nothing at all on that bag -- measured, not assumed:
// asking it for /camera/image_raw, a topic this walk reads without trouble, also fails with
// "no PointCloud2 message".  So the raw outlet belongs here, next to the walk that works.
bool BagReader::readRaw(const std::string& path, const std::string& topic, std::size_t max_messages,
                        const std::function<bool(double, const std::uint8_t*, std::size_t)>& on_message,
                        std::size_t* delivered, std::string* error) {
  std::vector<std::uint8_t> data;
  if (!loadWholeFile(path, &data, error)) {
    return false;
  }
  std::map<std::uint32_t, BagConnection> connections;
  std::map<std::uint32_t, std::size_t> counts;
  std::uint64_t chunks = 0;
  std::string chunk_error;

  // ChunkVisitor stores a POINTER to the callback, so it has to point at a stable object: a copy of
  // the caller's std::function, which lives for the whole call.
  std::function<bool(double, const std::uint8_t*, std::size_t)> callback = on_message;

  ChunkVisitor visitor;
  visitor.connections = &connections;
  visitor.counts = &counts;
  visitor.chunks = &chunks;
  visitor.chunk_error = &chunk_error;
  visitor.topic_filter = topic;
  visitor.max_frames = max_messages;
  visitor.on_raw = &callback;

  if (!walkBag(data.data(), data.size(), &visitor, error)) {
    return false;
  }
  if (delivered != nullptr) {
    *delivered = visitor.raw_delivered;
  }
  if (visitor.raw_delivered == 0) {
    if (error != nullptr) {
      if (!chunk_error.empty()) {
        *error = chunk_error;
      } else {
        *error = "no messages on topic '" + topic + "' in " + path;
      }
    }
    return false;
  }
  return true;
}

}  // namespace jo
