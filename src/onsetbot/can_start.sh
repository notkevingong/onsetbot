#!/bin/bash

# Set the bitrate for the can0 interface (e.g., 500 kbit/s)
sudo ip link set can0 type can bitrate 500000

# or using ip link:
sudo ip link set up can0

# show details/status
sudo ip -d -s link show can0