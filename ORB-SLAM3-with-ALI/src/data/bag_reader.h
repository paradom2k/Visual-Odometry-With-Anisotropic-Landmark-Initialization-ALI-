#pragma once

#include "common/types.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace jo {

/// A ROS1 bag v2 connection record.
struct BagConnection {
  std::uint32_t id = 0;
  std::string topic;
  std::string type;
  std::string message_definition;
};

/// Summary of one bag file, without decoding any payload.
struct BagInfo {
  bool valid = false;
  std::string path;
  std::vector<BagConnection> connections;
  /// Message count per connection id.
  std::vector<std::pair<std::uint32_t, std::size_t>> message_counts;
  std::uint64_t chunk_count = 0;
  std::uint64_t file_size = 0;
};

/// Minimal ROS1 bag v2 reader.
///
/// Written from the bag format rather than pulled from a library: the dataset
/// ships raw ".bag" files and the build must stay dependency free on Windows
/// with no ROS installed.  Only what the project needs is implemented:
///   - the bag header record (chunk and connection counts)
///   - connection records, both at the top level and inside chunks
///   - message records inside uncompressed chunks
/// Compressed chunks and nested message decoding beyond the image header are
/// deliberately not supported and are reported as such.
class BagReader {
 public:
  /// Cheap scan: connections, counts and sizes.
  static bool inspect(const std::string& path, BagInfo* out, std::string* error);

  /// Decode every compressed-image message whose topic matches `topic`.  When
  /// `topic` is empty the first image topic found is used.  `max_frames` of 0
  /// means "no limit".  Image dimensions are read from the embedded JPEG
  /// stream so the caller does not need an image library to validate the data.
  static bool readImages(const std::string& path, const std::string& topic,
                         std::size_t max_frames, std::vector<ImageFrame>* out,
                         std::string* error);

  /// The same, but starting at the `first_frame`-th matching message.  The
  /// messages before it are counted and dropped without copying their payload,
  /// which is what makes stepping through a long sequence affordable: the file is
  /// memory mapped and walked, but only the frames asked for are materialised.
  static bool readImages(const std::string& path, const std::string& topic,
                         std::size_t first_frame, std::size_t count,
                         std::vector<ImageFrame>* out, std::string* error);

  /// SUPPLIED -- the raw-payload sibling of `readImages`.  Same bag, same walk, same topic matching
  /// (an exact topic name when `topic` is given), but each matching message is handed to
  /// `on_message` as its own bytes instead of being decoded as an image.  `on_message` receives
  /// (stamp seconds, payload, payload size) and returns false to stop the walk.
  ///
  /// It exists for `tools/imu_dump.cpp`: the Imu topic is not an image topic, and the PointCloud2
  /// walker in src\lidar\lidar_bag.h delivers nothing on the 4Seasons recording -- including on
  /// /camera/image_raw, which this very walk reads without trouble.  Nothing in the image path is
  /// changed by it: the branch it uses is guarded on this callback being set.
  static bool readRaw(const std::string& path, const std::string& topic, std::size_t max_messages,
                      const std::function<bool(double, const std::uint8_t*, std::size_t)>& on_message,
                      std::size_t* delivered, std::string* error);
};

/// One image message's place in a bag, taken from the bag's own index.
struct BagImageLocation {
  double time = 0.0;                ///< message header stamp, seconds
  std::uint64_t chunk_data_offset = 0;  ///< file offset of the chunk's data
  std::uint32_t offset_in_chunk = 0;    ///< message offset inside that data
};

/// Every image message of one bag, located by the bag's index.
///
/// Why this exists as a separate path from readImages: a bag here is four
/// hundred megabytes of JPEGs and reading one costs a whole-file pass, which is
/// four to six seconds.  A live display reads one image per solve step and can
/// afford none of that.  The bag already carries an index -- one entry per
/// message with its time and its offset in its chunk -- so the reader can walk
/// the top-level records *seeking past* every chunk payload and come away with
/// the complete time base in tens of milliseconds, then fetch a single message by
/// seeking straight to it.  Measured on this dataset: 0.02 s to index a 456 MB
/// bag and 595 images, against 5 s to read it.
struct BagImageIndex {
  std::string path;
  std::string topic;
  std::uint32_t connection = 0;
  /// True when the bag's chunks are uncompressed, which is what the by-index
  /// reader requires.
  bool readable = false;
  /// In time order.  `chunk_data_offset + offset_in_chunk` is the message.
  std::vector<BagImageLocation> entries;
};

/// Read a bag's index for the image topic.  `topic` empty means "the first image
/// topic in the bag".  Does not read any message payload.
bool indexBagImages(const std::string& path, const std::string& topic,
                    BagImageIndex* out, std::string* error);

/// The same index, remembered between runs.
///
/// The walk above seeks past every chunk of a bag, which is tens of milliseconds when the
/// bag is in the OS cache and a few seconds when it is not -- this dataset's bags live on a
/// separate drive and are ~440 MB each, and the viewer indexes one every time the replay
/// crosses into it (37 of them over a whole drive).  The index itself is tiny: one line per
/// image, about 7 KB for a 600-image bag.  `cache_dir` empty means "no cache", and the walk
/// happens every time.  A cache file is only used when the bag it names is the same size it
/// was, so a re-recorded bag is re-indexed rather than mis-read.
bool indexBagImagesCached(const std::string& path, const std::string& topic,
                          const std::string& cache_dir, BagImageIndex* out,
                          std::string* error);

/// Materialise one image, located by an index entry.  Reads the message header
/// and its JPEG payload and nothing else.
bool readBagImage(const std::string& path, const BagImageIndex& index,
                  std::size_t frame, ImageFrame* out, std::string* error);

/// Read width, height and component count from a JPEG byte stream.  Returns
/// false when no start-of-frame marker is found.
bool jpegDimensions(const std::uint8_t* data, std::size_t size, int* width,
                    int* height, int* components);

}  // namespace jo
