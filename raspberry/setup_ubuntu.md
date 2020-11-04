## How to prepare Raspberry 

Download ubuntu server:

18.04.5 LTS: ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz

https://wiki.ubuntu.com/ARM/RaspberryPi

`xzcat ubuntu.img.xz | sudo dd bs=4M of=/dev/mmcblk0`

We need to edit some files on sdcard

`/etc/netplab/50-cloud-init.yaml`:
```
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            optional: true
            access-points:
                "wifi_name":
                    password: "password"
            dhcp4: true
```

write a file `/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg` with the following:

`network: {config: disabled}`


