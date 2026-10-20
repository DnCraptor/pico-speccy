#!/bin/sh
# defrag.sh — перезаписать файлы на FAT32, чтобы каждый лёг одним куском
# usage: defrag.sh FILE [FILE...]

[ $# -ge 1 ] || { echo "usage: $0 FILE [FILE...]" >&2; exit 1; }

for F in "$@"; do
    [ -f "$F" ] || { echo "$F: not a file" >&2; continue; }

    echo "== $F"
    echo "before: $(filefrag "$F" | sed 's/.*: //')"

    TMP=$(mktemp /tmp/defrag.XXXXXX) || exit 1
    cp -p "$F" "$TMP" && cmp -s "$F" "$TMP" || {
        echo "$F: copy to $TMP failed, file left untouched" >&2
        rm -f "$TMP"; continue
    }

    rm "$F" && sync
    if cp -p "$TMP" "$F" && sync && cmp -s "$TMP" "$F"; then
        rm -f "$TMP"
    else
        echo "$F: copy back failed, original kept in $TMP" >&2
        exit 1
    fi

    echo "after:  $(filefrag "$F" | sed 's/.*: //')"
done
