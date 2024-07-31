#!/bin/bash

g++ main.cpp -L./include -lmylibti5_multi_motor -L../include/can -lmylibscan -lcontrolcan -o multi_motor
