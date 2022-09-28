#!/bin/bash
aarch64-linux-gnu-g++ -std=c++20 -Wall -lgpiod -lgpiodcxx -lasound -L./libs -I./libs -O3 -o ./cnaf cnaf.cpp
