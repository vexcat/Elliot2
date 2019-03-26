#!/bin/bash
LINES_PER_PAGE=50
FIRST_FILE_BLENDS_WITH_PRETEXT=1
function file_ends_with_newline() {
    [[ $(tail -c1 "$1" | wc -l) -gt 0 ]]
}
OUTFILE=${1:-hardcopy.cpp}
echo Generating hardcopy to "$OUTFILE"
cat pretext.txt > $OUTFILE
PRELEN=$(cat "pretext.txt" | wc -l)
for filename in src/*; do
  if [ "$filename" == "src/cougarImage.c" ]; then continue; fi
  echo "//File $filename" >> $OUTFILE
  echo "" >> $OUTFILE
  cat "$filename" >> $OUTFILE
  if ! file_ends_with_newline $filename; then
    echo >> $OUTFILE
  fi
  LEN=$(sed -n '$=' "$filename")
  LEN=$(echo "$LEN" + 2 | bc)
  if [ "$FIRST_FILE_BLENDS_WITH_PRETEXT" == 1 ]; then
    FIRST_FILE_BLENDS_WITH_PRETEXT=0
    LEN=$(echo "$LEN" + "$PRELEN" | bc)
  fi
  echo $LEN
  REMAINING=$(echo "$LINES_PER_PAGE" - 1 - \("$LEN" + "$LINES_PER_PAGE" - 1\) % "$LINES_PER_PAGE" | bc)
  yes "" | head -n "$REMAINING" >> $OUTFILE
done
