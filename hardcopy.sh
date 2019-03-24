#!/bin/bash
LINES_PER_PAGE=70
function file_ends_with_newline() {
    [[ $(tail -c1 "$1" | wc -l) -gt 0 ]]
}
OUTFILE=${1:-hardcopy.cpp}
echo "Generating hardcopy to $OUTFILE"
cat pretext.txt > $OUTFILE
for filename in src/*; do
  if [ "$filename" == "src/cougarImage.c" ]; then continue; fi
  echo "" >> $OUTFILE
  echo "//File $filename" >> $OUTFILE
  echo "" >> $OUTFILE
  cat "$filename" >> $OUTFILE
  if ! file_ends_with_newline $filename; then
    echo >> $OUTFILE
  fi
done
