# moxa-irigb-driver

The IRIG-B driver is for moxa DA-820C embedded computer.
`/dev/moxa_irigb` is the entry point controlling the IRIG-B device.

1. Setting the path to your kernel source tree in Makefile.
To compile this driver, you should modify the KDIR to the path of your kernel source.
```
KDIR:=/lib/modules/`uname -r`/build
KERNEL_VERSION=`uname -r`
```

2. Compile the driver
```bash
root@Moxa:# make
```


3. Install and load the driver
You can install the driver
```bash
root@Moxa:# make install
```

Or install the driver manually
```bash
root@Moxa:# mkdir /lib/modules/`uname -r`/kernel/drivers/misc
root@Moxa:# cp -a moxa_irigb.ko /lib/modules/`uname -r`/kernel/drivers/misc/
```

and Load the driver manually
```bash
root@Moxa: insmod /lib/modules/`uname -r`/kernel/drivers/misc/moxa_irigb.ko
```

4. Load the driver in the initial script
```bash
load_irigb_driver() {
    if [ "`lsmod|grep -c moxa_irigb`" = "0" ]; then
        # If the driver has not been loaded, load it
        insmod /lib/modules/`uname -r`/kernel/drivers/misc/moxa_irigb.ko
    fi
}
```

6. IRIG-B tools
[moxa-irigb-tools](http://gitlab.syssw.moxa.com/MXcore-Package/moxa-irigb-tools)
