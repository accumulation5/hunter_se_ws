echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_N10P"' >/etc/udev/rules.d/100-wheeltec_lidar-N10P.rules

service udev reload
sleep 2
service udev restart


