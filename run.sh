git pull
sudo ntpd -q
echo "Completed ntpd -q"
make && ./single_chan_pkt_fwd
