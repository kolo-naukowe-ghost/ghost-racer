#!/bin/bash

echo "Killing gzserver"
pkill gzserver

echo "Killing gzclient"
pkill gzclient

sleep 1
echo "Killing xterm"
pkill xterm