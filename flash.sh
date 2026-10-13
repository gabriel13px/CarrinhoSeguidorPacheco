#!/usr/bin/env bash
# flash.sh — conveniência para compilar/gravar o firmware do carrinho (ESP32-S3) com PlatformIO.
#
#   ./flash.sh            compila, grava e abre o monitor serial (115200)
#   ./flash.sh build      só compila
#   ./flash.sh upload     compila e grava (sem monitor)
#   ./flash.sh monitor    só abre o monitor serial
#   ./flash.sh clean      limpa o build
#   ./flash.sh test       roda os testes do núcleo no PC (não toca no carrinho)
#
#   PORT=/dev/ttyACM0 ./flash.sh upload    força a porta (senão o PlatformIO detecta)
set -e
cd "$(dirname "$0")"

# acha o PlatformIO (comando pio, ou python -m platformio)
if command -v pio >/dev/null 2>&1; then PIO="pio"
elif command -v platformio >/dev/null 2>&1; then PIO="platformio"
elif python3 -c "import platformio" >/dev/null 2>&1; then PIO="python3 -m platformio"
else
  echo "PlatformIO não encontrado. Instale com:  pip install -U platformio"
  echo "(ou use a extensão PlatformIO IDE no VS Code)"
  exit 1
fi

PORT_ARG=""
[ -n "$PORT" ] && PORT_ARG="--upload-port $PORT --monitor-port $PORT"

case "${1:-all}" in
  build)   $PIO run ;;
  upload)  $PIO run -t upload $PORT_ARG ;;
  monitor) $PIO device monitor -b 115200 $PORT_ARG ;;
  clean)   $PIO run -t clean ;;
  test)    ./sim/build_core.sh ;;
  all)     $PIO run -t upload $PORT_ARG && echo ">> gravado. abrindo monitor (Ctrl+C sai)..." && $PIO device monitor -b 115200 $PORT_ARG ;;
  *)       echo "uso: ./flash.sh [build|upload|monitor|clean|test]"; exit 1 ;;
esac
