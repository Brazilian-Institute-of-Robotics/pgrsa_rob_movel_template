#!/bin/bash

for f in *.png *.jpg *.jpeg *.tga; do
  [ -e "$f" ] || continue
  base="${f%.*}"
  if [ ! -e "$base" ]; then
    ln -s "$f" "$base"
    echo "Created symlink: $base -> $f"
  fi
done