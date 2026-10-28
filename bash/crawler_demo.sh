#!/bin/sh
./build/arbotix_gui sketch/crawler_demo/build/MightyCore.avr.644/crawler_demo.ino.elf --link /tmp/ttyArbotiX0 -v --servo 1 --servo 2 --servo 3 --servo 4 --servo 5 --servo 6 --servo 7 --servo 8 --servo 9 --servo 10 --servo 11 --servo 12 --servo 13 --servo 14 --servo 15 --servo 16 --servo 17 --servo 18 2>&1 | tee /tmp/verbose.log
