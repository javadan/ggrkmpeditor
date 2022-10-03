#!/bin/bash

sudo cp robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start robot.service
sudo systemctl status robot.service
sudo systemctl enable robot.service




