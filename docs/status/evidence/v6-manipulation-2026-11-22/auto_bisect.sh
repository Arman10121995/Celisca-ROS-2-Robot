#!/bin/bash
# Binary search over bisect-revs.txt (oldest first): 34c82c7d passes, HEAD fails. Each step runs bisect_tiago.sh.
DIR=$(cd "$(dirname "$0")" && pwd); RUN=/workspace/molar/robot_lab_runtime/v6-manipulation-20261122/bisect
mapfile -t REVS < "$DIR/bisect-revs.txt"
lo=-1; hi=$((${#REVS[@]}-1))   # REVS[lo] passes (-1 = the Nov 19 base), REVS[hi] fails (HEAD)
while [ $((hi-lo)) -gt 1 ]; do
  mid=$(( (lo+hi)/2 )); rev=${REVS[$mid]}; tag=step-$mid-${rev:0:8}
  "$DIR/bisect_tiago.sh" "$rev" "$tag"
  passed=$(python3 -c "
import json;t=open('$RUN/$tag.json').read();d,_=json.JSONDecoder().raw_decode(t.lstrip()) if t.strip() else ({},0);print(bool(d.get('passed')))")
  echo "$mid $rev passed=$passed" >> "$DIR/bisect-progress.txt"
  if [ "$passed" = True ]; then lo=$mid; else hi=$mid; fi
done
echo "FIRST-BAD $hi ${REVS[$hi]}" >> "$DIR/bisect-progress.txt"
