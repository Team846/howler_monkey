echo "ON BOOT" > "/home/nvidia/apriltags/bootup.txt" &
sleep 5
sudo /usr/bin/python3.8 /home/nvidia/apriltags/test.py > "/home/nvidia/apriltags/log.txt" &
sudo /usr/bin/python3.8 /home/nvidia/apriltags/readAprilTag.py&
echo "FINISHED STARTUP" > "/home/nvidia/apriltags/finscript.txt"
