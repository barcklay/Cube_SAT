#!/bin/zsh
# Double-click in Finder: opens Terminal and shows once a second whether the
# NUCLEO board (ST-LINK virtual COM port) is plugged in. Close the window to stop.
printf '\e]0;HAB-1: плата\a'
while true; do
  if ls /dev/cu.usbmodem* >/dev/null 2>&1; then
    echo "$(date +%T)  ✅ плата подключена"
  else
    echo "$(date +%T)  ❌ платы нет"
  fi
  sleep 1
done
