#!/usr/bin/env bash
# Full monocular-inertial runs of the four configurations.  Sequential; do not edit while running.
set -u
cd "$(dirname "$0")/.."
PY="C:/Users/fengx/anaconda3/envs/fourseason/python.exe"

./bin/ali_run.exe --config conf/4seasons_neighborhood_3.conf     --out sol/rec_n3_base.txt > sol/rec_n3_base.log 2>&1
echo "n3 base $? ($(date +%H:%M:%S))"
./bin/ali_run.exe --config conf/4seasons_neighborhood_3_ali.conf --out sol/rec_n3_ali.txt  > sol/rec_n3_ali.log 2>&1
echo "n3 ali $? ($(date +%H:%M:%S))"
./bin/ali_run.exe --config conf/4seasons_office_loop_5.conf      --out sol/rec_of_base.txt > sol/rec_of_base.log 2>&1
echo "of base $? ($(date +%H:%M:%S))"
./bin/ali_run.exe --config conf/4seasons_office_loop_5_ali.conf  --out sol/rec_of_ali.txt  > sol/rec_of_ali.log 2>&1
echo "of ali $? ($(date +%H:%M:%S))"

for f in sol/rec_n3_base.txt sol/rec_n3_ali.txt; do
  printf "%s: " "$f"; "$PY" tools/gauge_free_score.py --est "$f" --ref conf/data/neighborhood_3_reference.csv --time-scale 1 2>&1 | grep -E "RMS 3D|path length"
done
for f in sol/rec_of_base.txt sol/rec_of_ali.txt; do
  printf "%s: " "$f"; "$PY" tools/gauge_free_score.py --est "$f" --ref conf/data/office_loop_5_reference.csv --time-scale 1 2>&1 | grep -E "RMS 3D|path length"
done
echo "=== mono runs done ==="
