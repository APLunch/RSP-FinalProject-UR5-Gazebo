#!/bin/bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3
sudo apt install -y python3-pip
sudo pip install pysocks
sudo pip install requests
sudo pip install openai 
exit 0
