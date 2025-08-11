KVERSION ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVERSION)/build

modules:
	$(MAKE) -C $(KDIR) M=$$(pwd) modules

modules_install: modules
	$(MAKE) -C $(KDIR) M=$$(pwd) modules_install
	depmod -A
	install -m 644 99-vserial.rules /usr/lib/udev/rules.d

all: modules

clean:
	$(MAKE) -C $(KDIR) M=$$(pwd) clean
