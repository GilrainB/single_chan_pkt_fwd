#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Running from $DIR"

cd $DIR
git pull
sudo ntpd -q
echo "Completed ntpd -q"
make && ./single_chan_pkt_fwd
