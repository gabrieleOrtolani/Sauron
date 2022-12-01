#!/bin/bash

# ROS 2 Galactic Geochelone installation script
# Roberto Masocco <robmasocco@gmail.com>
# November 28, 2021

echo "Checking Universe repository..."
if [[ -z "$(apt-cache policy | grep universe)" ]]; then
	sudo apt install software-properties-common
	sudo add-apt-repository universe
fi

echo "Adding ROS 2 apt repository..."
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "Installing ROS 2 main packages..."
sudo apt install -y ros-galactic-desktop ros-galactic-eigen3-cmake-module ros-galactic-gazebo-ros-pkgs

echo "Installing secondary packages and tools..."
sudo apt install -y python3-colcon-argcomplete
sudo pip3 install -U empy pyros-genmsg setuptools testresources
sudo apt install -y python3-colcon-common-extensions libasio-dev libtinyxml2-dev python3-rosdep

# IGMP firewall configuration
if ! command -v ufw &> /dev/null
then
  >&2 echo "ufw not found, skipping firewall configuration"
  echo "You might need to allow IGMP packets in iptables if you configured it to filter packets."
  echo "In case you install ufw or another firewall later on, please come back to this script and execute these last steps manually."
  echo "Otherwise, ROS 2 transmissions might not work."
  exit
fi
echo "Allowing IGMP packets in system firewall..."
# ref.: https://einar.slaskete.net/2014/05/03/allow-multicast-and-igmp-with-ufw-for-iptv-to-work/
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
LINES=$(sudo cat /etc/ufw/before.rules | wc -l)
ADD_LINE=$((LINES-2))
sudo sed -i "$ADD_LINE a # allow IGMP\n-A ufw-before-input -p igmp -d 224.0.0.0/4 -j ACCEPT\n-A ufw-before-output -p igmp -d 224.0.0.0/4 -j ACCEPT\n" /etc/ufw/before.rules
sudo ufw reload

