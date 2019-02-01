#  dummy raw1394 kernel module Makefile
#  see readme.txt for information on how to use it

obj-m += raw1394.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
