#!/bin/bash
sudo systemctl stop ArtinxHub.service
sudo kill $(pidof ArtinxHub)