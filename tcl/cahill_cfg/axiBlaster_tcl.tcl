#jyclone V SocKit board
# http://www.altera.com/b/arrow-sockit.html
#
# Software support page:
# http://www.rocketboards.org/

# openocd does not currently support the on-board USB Blaster II.
# Install the JTAG header and use a USB Blaster insteadndto 0.0.0.0.
bindto 0.0.0.0
adapter driver axi_blaster
#axi_blaster axiBlaster_register_addr 0x43c40000 0x43c40004 0x43c40008 0x43c4000c 0x43c40010
axi_blaster axiBlaster_register_addr 0x43c30000 0x43c30004 0x43c30008 0x43c3000c 0x43c30010
#source [find target/altera_fpgasoc.cfg]
transport select jtag
# If the USB Blaster II were supported, these settings would be needed
#usb_blaster vid_pid 0x09fb 0x6810
#usb_blaster device_desc "USB-Blaster II"

adapter speed 100
