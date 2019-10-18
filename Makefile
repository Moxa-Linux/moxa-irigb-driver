TARGET	:= moxa_irigb
KVER	:= $(shell uname -r)
KDIR	:= /lib/modules/$(KVER)/build
PWD	:= $(shell pwd)

obj-m += moxa_irigb.o

all: modules

modules:
	@echo "Making modules $(TARGET).ko ..."
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install: modules
	/usr/bin/install -m 644 -D $(TARGET).ko /lib/modules/$(KVER)/kernel/drivers/misc/$(TARGET).ko
	/usr/bin/install -m 644 -D $(TARGET).conf /usr/lib/modules-load.d/$(TARGET).conf

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

