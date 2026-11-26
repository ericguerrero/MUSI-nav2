#!/bin/sh
# Runs inside the Lichtblick container: docker exec -i musi_lichtblick sh < patch-lichtblick.sh
#
# Lichtblick 2.5.2: during page load the 3D panel's effect cleanup unadvertises
# its click-to-publish topics and they are not advertised again, so a goal drawn
# on the map is never sent. This removes that cleanup.
#
# The patched chunk gets a different file name, and so does the main bundle that
# names it. The server sends no Cache-Control, so a browser that already holds
# the original files would otherwise keep running them for days.
set -e
cd /src

needle='e.unadvertise?.(lt.goal),e.unadvertise?.(lt.point),e.unadvertise?.(lt.pose)'
chunk=$(grep -l -F "$needle" [0-9]*.js | head -n 1) || true
if [ -z "$chunk" ]; then
    echo "lichtblick patch: cleanup code not found, nothing changed"
    exit 0
fi

id=${chunk%%.*}
hash=${chunk#*.}
hash=${hash%.js}
main=$(ls main.*.js | head -n 1)
main_hash=${main#main.}
main_hash=${main_hash%.js}

sed "s/()=>{$needle}/()=>{}/" "$chunk" > "$id.${hash}p1.js"
sed "s/$id:\"$hash\"/$id:\"${hash}p1\"/" "$main" > "main.${main_hash}p1.js"
sed -i "s/main\.$main_hash\.js/main.${main_hash}p1.js/" index.html
echo "lichtblick patch: $chunk -> $id.${hash}p1.js"
