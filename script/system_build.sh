#!/bin/bash

# System Build Script for Eurobot 2026 SIMA
# This script sets up the development environment with necessary packages and configurations

set -e  # Exit on any error

echo "=== Starting System Build Setup ==="

# Function to check if running as root
check_root() {
    if [[ $EUID -eq 0 ]]; then
        echo "Error: This script should not be run as root (except for sudo commands)"
        exit 1
    fi
}

# Check if not running as root
check_root

echo "=== Updating System Packages ==="
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y
sudo apt autoclean -y

# Backup the config file
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.backup.$(date +%Y%m%d_%H%M%S)

# Check if fan settings already exist
echo "" | sudo tee -a /boot/firmware/config.txt
echo "# Fan configuration added by system_build.sh" | sudo tee -a /boot/firmware/config.txt
echo "dtparam=cooling_fan=on" | sudo tee -a /boot/firmware/config.txt
echo "dtparam=fan_temp3=30000,fan_temp3_hyst=5000,fan_temp3_speed=255" | sudo tee -a /boot/firmware/config.txt
echo "Fan configuration added to config.txt"

# Enable UART
echo "=== Enabling UART ==="
echo "" | sudo tee -a /boot/firmware/config.txt
echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
echo "dtoverlay=uart0" | sudo tee -a /boot/firmware/config.txt
echo "UART enabled in config.txt"


echo "=== Installing Docker ==="
# Remove old Docker packages
echo "Removing old Docker packages..."
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do 
    sudo apt-get remove -y $pkg 2>/dev/null || true
done

# Add Docker's official GPG key
echo "Adding Docker's official GPG key..."
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources
echo "Adding Docker repository..."
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
echo "Installing Docker..."
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add user to docker group
echo "Adding user $USER to docker group..."
sudo usermod -aG docker $USER