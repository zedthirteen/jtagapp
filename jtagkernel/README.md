see: https://matthewarcus.wordpress.com/2018/01/27/using-the-cycle-counter-registers-on-the-raspberry-pi-3/

just use "make" to build

then (to test it works):

sudo insmod enable_ccr.ko


then:

dmesg | tail 

to show it loaded


To make it permanently available:
=================================
see: https://stackoverflow.com/questions/4356224/how-to-load-a-custom-module-at-the-boot-time-in-ubuntu

sudo cp enable_ccr.ko /lib/modules/$(uname -r)/kernel/drivers/

echo 'enable_ccr' | sudo tee -a /etc/modules

sudo depmod

sudo reboot


