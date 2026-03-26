TARGET = dvorak
CC     = gcc
CFLAGS = -Wall -O3 -static

.PHONY: default all clean run test install uninstall

default: all

all: dvorak.c
	$(CC) $(CFLAGS) -o $(TARGET) dvorak.c

run:
	sudo ./dvorak -d /dev/input/by-id/usb-SONiX_USB_DEVICE-event-kbd


	$(CC) $(CFLAGS) -DDVORAK_TEST -o test_dvorak test_dvorak.c
	./test_dvorak

clean:
	-rm -f $(TARGET) test_dvorak

install: all
	cp dvorak /usr/local/bin/dvorak
	cp 80-dvorak.rules /etc/udev/rules.d/
	cp dvorak@.service /etc/systemd/system/
	# Create the dvorak system user in the input group if it doesn't exist
	id -u dvorak &>/dev/null || useradd -r -G input -s /sbin/nologin dvorak
	systemctl daemon-reload
	udevadm control --reload
	# Trigger udev for keyboards already plugged in
	udevadm trigger --subsystem-match=input --property-match=ID_INPUT_KEYBOARD=1 --action=add

uninstall:
	systemctl stop 'dvorak@*.service'
	rm -f /usr/local/bin/dvorak
	rm -f /etc/udev/rules.d/80-dvorak.rules
	rm -f /etc/systemd/system/dvorak@.service
	systemctl daemon-reload
	udevadm control --reload
