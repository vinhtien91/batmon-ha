# List BT Controllers

```
bluetoothctl list
ls -lA /sys/class/bluetooth/
hcitool  dev
rfkill
```

# Scan
```
hcitool -i hci0 inq

bluetoothctl
select <mac>
scan le
```