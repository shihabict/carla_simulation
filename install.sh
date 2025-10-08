#!/bin/bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.9 python3.9 - venv python3.9-dev

python3.9 -m venv carlaenv

source carlaenv/bin/activate

pip install -r requirements.txt