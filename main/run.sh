#!/bin/bash

rm main
g++ main.cpp -o main `pkg-config --cflags --libs opencv` -lpthread
./main