#!/usr/bin/env bash
# Give the VM's robot-facing network card an address on the Go2's network, then ping the dog.
#   bash robot_net_up.sh enp0s2
# The Go2's onboard computer is 192.168.123.161. The course guide uses 192.168.123.222/24 for the PC;
# that address is kept for the Mac's own adapter, so the VM takes 192.168.123.223 and the two never clash.
set -e
IF="${1:?usage: robot_net_up.sh <interface bridged to the robot, see: ip -br link>}"
sudo ip link set "$IF" up
sudo ip addr flush dev "$IF"
sudo ip addr replace 192.168.123.223/24 dev "$IF"
ip -br addr show "$IF"
echo "pinging the robot..."
ping -c 3 -W 1 192.168.123.161 && echo "ROBOT REACHABLE on $IF" || { echo "no reply: check the cable, the UTM bridged interface and that the robot is on"; exit 3; }
