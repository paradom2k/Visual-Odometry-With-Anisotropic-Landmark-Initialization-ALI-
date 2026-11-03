"""Walk a ROS bag v2.0 file and report its structure (topics, types, counts, time span, compression).

Pure Python, no rosbag/rosbags needed: it parses the record framing from the spec (13-byte version line,
then records of `uint32 length`, a header of `uint32 field-length + name=value` pairs, and a payload).
Chunk payloads are decompressed with what the standard library and the local environment provide: `none` and
`bz2` always, `lz4` if the module is installed (ROS's usual choice) -- and if lz4 is missing the script says
so instead of failing obscurely.

    python tools\\bag_inspect.py <bag> [--max-chunks N]
"""
import argparse
import bz2
import struct
import sys
from collections import Counter, OrderedDict

try:
    import lz4.frame as lz4_frame  # type: ignore
except Exception:  # pragma: no cover - depends on the environment
    lz4_frame = None


INT_FIELDS = {"conn", "size", "chunk_count", "conn_count", "index_pos"}


def decode_value(name, raw):
    """Bag header values are a mix of text and binary integers -- decode accordingly.

    Not doing this is why the first working walker still dispatched nothing: `conn` is a 4-byte little-endian
    integer, so the key stored from a connection record and the key looked up from a message record only
    matched by luck with the 'replace'-decoded text form.
    """
    if name == "t" and len(raw) == 8:
        seconds, nanoseconds = struct.unpack("<II", raw)
        return seconds + nanoseconds * 1e-9
    if name in INT_FIELDS and len(raw) in (1, 2, 4, 8):
        return int.from_bytes(raw, "little")
    return raw.decode("utf-8", "replace")


def read_header(blob):
    """Header fields of a record: repeated `uint32 len` + `name=value`."""
    fields = OrderedDict()
    offset = 0
    while offset < len(blob):
        (length,) = struct.unpack_from("<I", blob, offset)
        offset += 4
        entry = blob[offset:offset + length]
        offset += length
        name, _, value = entry.partition(b"=")
        fields[name.decode("utf-8", "replace")] = decode_value(name.decode("utf-8", "replace"), value)
    return fields


def decompress(algorithm, payload):
    if algorithm == "none" or algorithm == "":
        return payload
    if algorithm == "bz2":
        return bz2.decompress(payload)
    if algorithm == "lz4":
        if lz4_frame is None:
            raise RuntimeError("chunk uses lz4 but the 'lz4' module is not installed (pip install lz4)")
        return lz4_frame.decompress(payload)
    raise RuntimeError(f"unknown chunk compression '{algorithm}'")


def walk(stream, connections, counts, times, chunks, max_chunks):
    """Iterate a record stream (whole file, or one decompressed chunk).

    Record framing, confirmed against the bytes of a G-drive camera bag rather than from memory:

        uint32 header_len | header(header_len) | uint32 data_len | data(data_len)

    The earlier version of this walked `uint32 n | n bytes` and therefore read the *header* length as the
    whole record; and the file header record pads its own `data` (4027 bytes here), which is why the first
    chunk starts at 4117 -- not at 4096, and not right after the version line.
    """
    while True:
        raw = stream.read(4)
        if len(raw) < 4:
            return
        (header_length,) = struct.unpack("<I", raw)
        header_blob = stream.read(header_length)
        if len(header_blob) < header_length:
            return
        raw = stream.read(4)
        if len(raw) < 4:
            return
        (data_length,) = struct.unpack("<I", raw)
        payload = stream.read(data_length)
        if len(payload) < data_length:
            return
        header = read_header(header_blob)
        # `op` is a single *byte* in a bag header (0x02 message, 0x05 chunk, 0x07 connection, 0x03 file
        # header, 0x04 index data, 0x06 chunk info), not the text "0x05" -- comparing against the text (as
        # the earlier version did) matches nothing at all, which is exactly the "0 chunks, 0 connections"
        # symptom.
        kind = header.get("op", "")
        if kind == "\x07":  # connection
            topic = header.get("topic", "?")
            connections[header.get("conn")] = (topic, payload.split(b"=", 1)[0].decode("utf-8", "replace"))
        elif kind == "\x05":  # chunk
            chunks.append(header.get("compression", "none"))
            if len(chunks) <= max_chunks:
                import io
                walk(io.BytesIO(decompress(header.get("compression", "none"), payload)),
                     connections, counts, times, chunks, max_chunks + 1)
        elif kind == "\x02":  # message
            conn = header.get("conn")
            topic = connections.get(conn, ("?", ""))[0]
            counts[topic] += 1
            stamp = None
            for key in ("t", "time"):
                if key in header:
                    try:
                        stamp = int(header[key]) / 1e9
                    except ValueError:
                        stamp = None
            if stamp is not None:
                low, high = times.get(topic, (stamp, stamp))
                times[topic] = (min(low, stamp), max(high, stamp))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--max-chunks", type=int, default=2)
    args = parser.parse_args(argv)

    with open(args.bag, "rb") as handle:
        version = handle.readline()
        # Walking starts right after the version line: the file-header record itself carries the 4 KB of
        # padding in its `data`, so the first chunk lands after it (4117 in the G-drive camera bags).
        print(f"{args.bag}\n  header: {version.strip().decode('utf-8', 'replace')}")
        connections, counts, times, chunks = {}, Counter(), {}, []
        walk(handle, connections, counts, times, chunks, args.max_chunks)

    print(f"  chunk record(s) seen: {len(chunks)}, compressions: {sorted(set(chunks))}")
    if "lz4" in chunks and lz4_frame is None:
        print("  NOTE: lz4 chunks present and the lz4 module is missing -> pip install lz4")
    print(f"  {len(connections)} connection(s):")
    for conn, (topic, type_name) in sorted(connections.items(), key=lambda kv: kv[1][0]):
        span = times.get(topic)
        when = f", {span[0]:.3f}..{span[1]:.3f} s" if span else ""
        print(f"    conn {conn:>3}  {topic:<34} {type_name:<28} {counts.get(topic, 0):>6} msg{when}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
