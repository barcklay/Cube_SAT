#!/bin/zsh
# Double-click in Finder: runs the lift test of the flight state machine (HW-18).
# Board on USB; nothing else may read the port (close telemetry.py first).
printf '\e]0;HAB-1: тест в лифте\a'
cd "$(dirname "$(readlink "$0" || echo "$0")")/.." || exit 1
if ! ls /dev/cu.usbmodem* >/dev/null 2>&1; then
  echo "❌ платы нет: вставь USB и запусти ещё раз"
  read -r "?Enter — закрыть окно"
  exit 1
fi
if lsof /dev/cu.usbmodem* >/dev/null 2>&1; then
  echo "❌ порт занят другой программой (закрой telemetry.py) и запусти ещё раз"
  read -r "?Enter — закрыть окно"
  exit 1
fi
echo "Маршрут: вниз, 30 с на 1-м → вверх домой, 30 с → снова вниз, 2 мин внизу → домой."
echo "Крышку ноутбука не закрывать. События будут появляться ниже."
echo
python3 tools/lift_test.py
echo
echo "Готово. Лог: $(ls -t lift_test_*.log 2>/dev/null | head -1)"
read -r "?Enter — закрыть окно"
