#!/bin/bash

echo "Installing robot launch service to run on boot..."

# Make sure launch.sh is executable
chmod +x launch.sh

# Copy service file to systemd directory
sudo cp robot-launch.service /etc/systemd/system/

# Reload systemd to recognize the new service
sudo systemctl daemon-reload

# Enable the service to start on boot
sudo systemctl enable robot-launch.service

# Check service status
echo "Service installed! Current status:"
sudo systemctl status robot-launch.service

echo ""
echo "To start the service now:"
echo "  sudo systemctl start robot-launch.service"
echo ""
echo "To stop the service:"
echo "  sudo systemctl stop robot-launch.service"
echo ""
echo "To disable auto-start on boot:"
echo "  sudo systemctl disable robot-launch.service"
echo ""
echo "To view service logs:"
echo "  sudo journalctl -u robot-launch.service -f"