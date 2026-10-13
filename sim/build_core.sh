#!/usr/bin/env bash
# Compila o núcleo (lib/core) como biblioteca compartilhada p/ o simulador e roda os testes nativos.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
mkdir -p "$HERE/build"
CXX="${CXX:-g++}"
FLAGS="-std=c++11 -O2 -Wall -Wextra -I$ROOT/lib/core"
$CXX $FLAGS -fPIC -shared "$ROOT/lib/core/lf_core.cpp" -o "$HERE/build/liblfcore.so"
$CXX $FLAGS "$ROOT/lib/core/lf_core.cpp" "$ROOT/test/test_core/test_main.cpp" -o "$HERE/build/test_core"
echo "build ok -> $HERE/build/"
if [ "${1:-}" != "--no-test" ]; then "$HERE/build/test_core"; fi
