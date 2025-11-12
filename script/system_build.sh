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

# Function to prompt for user input
prompt_user_config() {
    echo "=== Git Configuration Setup ==="
    read -p "Enter your Git username: " GIT_USERNAME
    read -p "Enter your Git email: " GIT_EMAIL
}

# Check if not running as root
check_root

# Prompt for user configuration
prompt_user_config

echo "=== Updating System Packages ==="
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y
sudo apt autoclean

echo "=== Installing OpenSSH Server ==="
sudo apt install -y openssh-server
sudo systemctl enable ssh
echo "SSH server installed and enabled"

echo "=== Installing Git and Network Tools ==="
sudo apt install -y git net-tools

echo "=== Configuring Git ==="
git config --global user.name "$GIT_USERNAME"
git config --global user.email "$GIT_EMAIL"
echo "Git configured with username: $GIT_USERNAME and email: $GIT_EMAIL"

echo "=== Configuring Fan Settings ==="
echo "Adding fan configuration to /boot/firmware/config.txt"
echo "Please note: You may need to manually edit /boot/firmware/config.txt if this fails"

# Backup the config file
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.backup.$(date +%Y%m%d_%H%M%S)

# Check if fan settings already exist
if ! grep -q "dtparam=cooling_fan=on" /boot/firmware/config.txt; then
    echo "" | sudo tee -a /boot/firmware/config.txt
    echo "# Fan configuration added by system_build.sh" | sudo tee -a /boot/firmware/config.txt
    echo "dtparam=cooling_fan=on" | sudo tee -a /boot/firmware/config.txt
    echo "dtparam=fan_temp3=36000,fan_temp3_hyst=5000,fan_temp3_speed=255" | sudo tee -a /boot/firmware/config.txt
    echo "Fan configuration added to config.txt"
else
    echo "Fan configuration already exists in config.txt"
fi

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

echo "=== System Build Setup Complete ==="
echo ""
echo "IMPORTANT NOTES:"
echo "1. You need to log out and log back in (or reboot) for Docker group changes to take effect"
echo "2. Fan settings have been added to /boot/firmware/config.txt - a reboot is required for these to take effect"
echo "3. A backup of your original config.txt has been created"
echo "4. SSH server is now enabled and running"
echo "5. Git has been configured with your provided credentials"
echo ""
echo "To verify Docker installation after relogging, run: docker --version"
echo "To verify Docker group membership, run: groups"
echo ""
echo "Setup completed successfully!"