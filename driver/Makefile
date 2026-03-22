obj-m := servo_driver.o

KDIR := /usr/src/linux-headers-5.10.160-rockchip-rk3588
PWD  := $(shell pwd)

all:
        make -C $(KDIR) M=$(PWD) modules

clean:
        make -C $(KDIR) M=$(PWD) clean
