TARGET	:= moxa_irigb
KVER	:= $(shell uname -r)
KDIR	:= /lib/modules/$(KVER)/build
PWD	:= $(shell pwd)

obj-m += moxa_irigb.o

all: modules

modules:
	@echo "Making modules $(TARGET).ko ..."
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install: all
	mkdir -v -p "$(DESTDIR)/lib/modules/$(KVER)/kernel/drivers/misc"
	install $(TARGET).ko $(DESTDIR)/lib/modules/$(KVER)/kernel/drivers/misc/

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

