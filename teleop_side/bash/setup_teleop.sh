#!/bin/bash

# Setup script for teleoperation side
# This script handles:
# 1. Setting permissions for damiao usb2can module

set -e

echo "=========================================="
echo "Teleoperation Side Setup"
echo "=========================================="

# Setup damiao usb2can module permissions
echo ""
echo "Setting permissions for damiao usb2can module..."
sudo chmod -R 777 /dev/ttyACM* || {
    echo "Warning: Could not set permissions for /dev/ttyACM*. Make sure the device is connected."
}

echo ""
echo "=========================================="
echo "Teleoperation Side Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Setup CAN ports manually:"
echo "   cd <piper_ros>"
echo "   bash find_all_can_port.sh"
echo "   bash can_activate.sh can_left 1000000 \"1-4.3:1.0\""
echo "   bash can_activate.sh can_right 1000000 \"1-4.4:1.0\""
echo ""
echo "2. Launch teleoperation nodes:"
echo "   roslaunch teleop_bridge start_teleop_all.launch left_can_port:=can_left right_can_port:=can_right"
echo ""
