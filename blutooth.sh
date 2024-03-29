hciconfig hci0 down
sudo rmmod btusb
sudo modprobe ntusb
hciconfig hci0 up
