## CANNOT CONNECT ISSUE 
Check user usb permissions
```bash 
# Check usb permissions
ls -l /dev/ttyACM0
# if your user name is not in permissions add yourself 
sudo usermod -a -G dialout $USER
# Apply changes 
newgrp dialout
# Check your now in the dialout permissions
groups $USER

```