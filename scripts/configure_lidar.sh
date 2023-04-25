#!/bin/bash
sudo route delete 192.168.191.254 enp48s0
sudo ifconfig enp48s0 192.168.191.1
sudo route add 192.168.191.254 enp48s0