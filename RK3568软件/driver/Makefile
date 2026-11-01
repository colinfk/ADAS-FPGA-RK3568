obj-m += adas_video_pcie.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

.PHONY: player
player: play_adas_native

play_adas_native: play_adas_native.c
	$(CC) -O2 -std=c11 -Wall -Wextra -Werror $< -o $@

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
