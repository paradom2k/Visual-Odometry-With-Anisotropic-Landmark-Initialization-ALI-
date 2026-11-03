"""Build the two ali-vins image manifests from the inputs already on this machine.

The manifests are plain "<timestamp> <image path>" lines, which is what the driver and the UI
expect.  Paths are absolute because the datasets live outside the project.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "config" / "data"

# neighborhood_3 == recording_2020-10-07_14-53-52.  The already-extracted fs_full frames are the
# same sequence; reuse them rather than copying 9.5k PNGs.
src = Path(r"E:/jo_gvil/jo_gvil 1.0/build/inputs/fs_full.txt")
out = DATA / "neighborhood_3_manifest.txt"
prefix_old = "build\\inputs\\fs_full\\frames"
prefix_new = "E:/jo_gvil/jo_gvil 1.0/build/inputs/fs_full/frames"
lines = []
for line in src.read_text().splitlines():
    if not line or line.startswith("#"):
        continue
    line = line.replace(prefix_old, prefix_new).replace("\\", "/")
    lines.append(line)
out.write_text("\n".join(lines) + "\n")
print(f"neighborhood_3: {len(lines)} frames -> {out}")

# office_loop_5 == recording_2021-01-07_12-04-03.  Its existing manifest is correct except that
# the recording root was D: when it was generated on the older machine.
src = Path(r"E:/jo_gvil/recover_jo/e1/out/b/4seasons_office.txt")
out = DATA / "office_loop_5_manifest.txt"
text = src.read_text().replace("D:/driving dataset", "E:/driving dataset")
out.write_text(text)
print(f"office_loop_5: {sum(1 for l in text.splitlines() if l and not l.startswith('#'))} frames -> {out}")
