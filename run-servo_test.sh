#!/bin/bash
gcc -Wall -pthread -o servo_test ArduinoPiGpio.cpp servo_test.cpp -lpigpio -lrt
sudo ./servo_test
exit 0
