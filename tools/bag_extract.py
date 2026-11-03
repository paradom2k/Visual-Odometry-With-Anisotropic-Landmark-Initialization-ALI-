"""Extract route E's inputs from a ROS bag: frames as PNG plus times.csv, and IMU as CSV.

Pure Python (numpy + Pillow), which is what the workspace has: no cv2, no rosbag, no network.  The record
framing below was derived from the actual bytes of both datasets' bags (`tools/bag_inspect.py`), and both use
**uncompressed** chunks -- so nothing here needs lz4.

Message layouts implemented (the two the datasets actually publish):

    sensor_msgs/Image             header, height, width, encoding, is_bigendian, step, data
    sensor_msgs/CompressedImage   header, format, data            (JPEG -> Pillow)
    sensor_msgs/Imu               header, orientation(4), cov(9), angular_velocity(3), cov(9),
                                  linear_acceleration(3), cov(9)

`std_msgs/Header` is `uint32 seq | uint32 sec | uint32 nsec | string frame_id`.

    python tools\\bag_extract.py --bag <bag> --kind fourseasons|i300 --out build\\inputs\\w1 \\
        [--start S --duration D] [--image-topic T] [--imu-topic T]
"""
import argparse
import os
import struct
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_inspect import decompress, read_header  # noqa: E402  (reuse the verified framing)


class Cursor:
    """Little-endian reader over a payload."""

    def __init__(self, blob):
        self.blob = blob
        self.offset = 0

    def u32(self):
        (value,) = struct.unpack_from("<I", self.blob, self.offset)
        self.offset += 4
        return value

    def i32(self):
        (value,) = struct.unpack_from("<i", self.blob, self.offset)
        self.offset += 4
        return value

    def f64(self):
        (value,) = struct.unpack_from("<d", self.blob, self.offset)
        self.offset += 8
        return value

    def string(self):
        length = self.u32()
        text = self.blob[self.offset:self.offset + length].decode("utf-8", "replace")
        self.offset += length
        return text

    def bytes(self, count):
        data = self.blob[self.offset:self.offset + count]
        self.offset += count
        return data

    def header_stamp(self):
        self.u32()                     # seq
        seconds = self.u32()
        nanoseconds = self.u32()
        self.string()                  # frame_id
        return seconds + nanoseconds * 1e-9


def image_to_array(payload, compressed):
    """`sensor_msgs/Image`/`CompressedImage` payload -> 8-bit grayscale or RGB numpy array."""
    cursor = Cursor(payload)
    stamp = cursor.header_stamp()
    if compressed:
        cursor.string()                                   # format, e.g. "jpeg"
        data = cursor.bytes(cursor.u32())
        import io
        image = Image.open(io.BytesIO(data))
        return stamp, np.asarray(image.convert("L"))
    height = cursor.u32()
    width = cursor.u32()
    encoding = cursor.string()
    # is_bigendian is a single uint8, not a uint32: reading four bytes here shifts step and data and
    # produces a reshape error like 'cannot reshape array of size 319997 into shape (400, 81920000)'.
    cursor.bytes(1)
    step = cursor.u32()
    data = cursor.bytes(cursor.u32())
    raw = np.frombuffer(data, dtype=np.uint8)
    if encoding in ("mono8", "8UC1"):
        return stamp, raw.reshape(height, step)[:, :width].copy()
    if encoding in ("rgb8", "8UC3"):
        return stamp, raw.reshape(height, step)[:, :width * 3].reshape(height, width, 3)[:, :, ::-1].copy()
    if encoding in ("bgr8",):
        return stamp, raw.reshape(height, step)[:, :width * 3].reshape(height, width, 3).copy()
    raise RuntimeError(f"unsupported image encoding '{encoding}'")


def imu_sample(payload):
    """`sensor_msgs/Imu` payload -> (stamp, gyro, accel), both in SI units."""
    cursor = Cursor(payload)
    stamp = cursor.header_stamp()
    for _ in range(4):
        cursor.f64()                                      # orientation
    cursor.offset += 9 * 8                                # orientation covariance
    gyro = (cursor.f64(), cursor.f64(), cursor.f64())
    cursor.offset += 9 * 8
    accel = (cursor.f64(), cursor.f64(), cursor.f64())
    return stamp, gyro, accel


STATS = {"chunks": 0, "messages": 0, "dispatched": 0, "unmatched": 0, "seen_topics": set()}


def walk(stream, handlers, connections, depth=0):
    """Walk records, recording connections and dispatching message payloads to `handlers[topic]`.

    The connection map persists across chunks on purpose: a message in one chunk may name a connection that
    was declared in an earlier one.  The counters exist because "0 frames extracted" has three very different
    causes -- the walk stopping early, no message records inside the chunks, or a topic mismatch -- and one
    run should say which.
    """
    while True:
        raw = stream.read(4)
        if len(raw) < 4:
            break
        (header_length,) = struct.unpack("<I", raw)
        header_blob = stream.read(header_length)
        raw = stream.read(4)
        if len(raw) < 4:
            break
        (data_length,) = struct.unpack("<I", raw)
        payload = stream.read(data_length)
        if len(payload) < data_length:
            break
        header = read_header(header_blob)
        kind = header.get("op", "")
        if kind == "\x07":
            connections[header.get("conn")] = header.get("topic", "")
        elif kind == "\x05":
            STATS["chunks"] += 1
            import io
            walk(io.BytesIO(decompress(header.get("compression", "none"), payload)),
                 handlers, connections, depth + 1)
        elif kind == "\x02":
            STATS["messages"] += 1
            topic = connections.get(header.get("conn"), "")
            STATS["seen_topics"].add(topic)
            handler = handlers.get(topic)
            if handler is None:
                STATS["unmatched"] += 1
            else:
                STATS["dispatched"] += 1
                handler(topic, payload)
    if depth == 0:
        print(f"  walk stats: {STATS['chunks']} chunk(s), {STATS['messages']} message record(s), "
              f"{STATS['dispatched']} dispatched, {STATS['unmatched']} unmatched; "
              f"topics seen: {sorted(t for t in STATS['seen_topics'] if t)}")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--kind", choices=("fourseasons", "i300"), default="fourseasons")
    parser.add_argument("--out", required=True)
    parser.add_argument("--start", type=float, default=0.0)
    # `--duration 0` means "no window".  A finite default (1e9) looked generous but silently rejected every
    # frame of the G-drive bags, whose stamps are ~1.4e9 (Unix-ish): 601 messages dispatched, 0 written, no
    # error -- which is exactly the symptom that took two rounds to pin down.
    parser.add_argument("--duration", type=float, default=0.0)
    # Bag clocks differ from the project's seconds-of-week: I300 needs -1404950399.8 to land in SOW.
    parser.add_argument("--clock-offset", type=float, default=0.0)
    parser.add_argument("--image-topic", default="")
    parser.add_argument("--imu-topic", default="")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args(argv)

    defaults = {
        "fourseasons": ("/camera/image_raw", "/imu_raw", False),
        "i300": ("/cam00/image_raw/compressed", "", True),
    }
    image_topic = args.image_topic or defaults[args.kind][0]
    imu_topic = args.imu_topic or defaults[args.kind][1]
    compressed = defaults[args.kind][2]

    frames_dir = os.path.join(args.out, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    times = []
    imu = []
    connections = {}
    end = args.start + args.duration if args.duration > 0.0 else float("inf")
    seen = [0]

    def on_image(topic, payload):
        stamp, pixels = image_to_array(payload, compressed)
        stamp += args.clock_offset
        if not (args.start <= stamp <= end):
            return
        if seen[0] % args.stride:
            seen[0] += 1
            return
        if args.max_frames and len(times) >= args.max_frames:
            return
        name = f"frame_{len(times):06d}.png"
        Image.fromarray(pixels).save(os.path.join(frames_dir, name))
        times.append((name, stamp))
        seen[0] += 1

    def on_imu(topic, payload):
        stamp, gyro, accel = imu_sample(payload)
        if args.start - 1.0 <= stamp <= end + 1.0:
            imu.append((stamp, gyro, accel))

    handlers = {image_topic: on_image}
    if imu_topic:
        handlers[imu_topic] = on_imu

    with open(args.bag, "rb") as handle:
        handle.readline()                     # "#ROSBAG V2.0"; records follow the file-header record
        walk(handle, handlers, connections)
        if not connections:
            print("  no connection record found -- is this a ROS bag v2.0 file?")
            return 1
        print("  connections: " + ", ".join(sorted(set(connections.values()))))

    with open(os.path.join(args.out, "times.csv"), "w", encoding="utf-8", newline="\n") as sink:
        sink.write("frame,stamp\n")
        for index, (name, stamp) in enumerate(times):
            sink.write(f"{index},{stamp:.6f}\n")
    if imu:
        with open(os.path.join(args.out, "imu.csv"), "w", encoding="utf-8", newline="\n") as sink:
            sink.write("# t_seconds,gx,gy,gz,ax,ay,az\n")
            for stamp, gyro, accel in imu:
                sink.write(f"{stamp:.9f}," + ",".join(f"{v:.9f}" for v in (*gyro, *accel)) + "\n")

    print(f"{args.bag}\n  {len(times)} frame(s) -> {frames_dir}\n  {len(imu)} IMU sample(s)"
          f"\n  image topic {image_topic}" + (f", imu topic {imu_topic}" if imu_topic else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
