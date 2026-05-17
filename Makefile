TARGET  = dvorak
CC      = gcc
CFLAGS  = -Wall -O3

PREFIX  ?= /usr/local
BINDIR  := $(PREFIX)/bin

# Path where 'cargo build --release' puts its output
RUST_BIN := target/release/$(TARGET)

.PHONY: default all rust clean run stop test install install-rust install-c uninstall

# Default: build the Rust binary (preferred)
default: rust

# Alias so plain 'make' and 'make all' both work
all: rust

# ── Build targets ─────────────────────────────────────────────────────────────

rust: $(RUST_BIN)

$(RUST_BIN): src/main.rs Cargo.toml
	cargo build --release

# C build (kept for reference / cross-checking)
c: dvorak.c
	$(CC) $(CFLAGS) -o $(TARGET) dvorak.c

# ── Run / stop ────────────────────────────────────────────────────────────────

run: rust
	sudo ./$(RUST_BIN) -d /dev/input/by-id/usb-SONiX_USB_DEVICE-event-kbd

stop:
	systemctl stop 'dvorak@*.service'

# ── Test (C only) ─────────────────────────────────────────────────────────────

test: dvorak.c
	$(CC) $(CFLAGS) -DDVORAK_TEST -o test_dvorak test_dvorak.c
	./test_dvorak

# ── Install ───────────────────────────────────────────────────────────────────

# install: builds Rust binary then installs everything
install: install-rust

install-rust: $(RUST_BIN) _install-common

install-c: c
	install -Dm755 $(TARGET) $(DESTDIR)$(BINDIR)/$(TARGET)
	$(MAKE) _install-common

_install-common:
	# Stop any running instances first
	-systemctl stop 'dvorak@*.service' 2>/dev/null || true
	# Install the binary
	install -Dm755 $(RUST_BIN) $(DESTDIR)$(BINDIR)/$(TARGET)
	# Install udev rule and service unit
	install -Dm644 80-dvorak.rules  /etc/udev/rules.d/80-dvorak.rules
	install -Dm644 dvorak@.service  /etc/systemd/system/dvorak@.service
	# Reload systemd and udev
	systemctl daemon-reload
	udevadm control --reload
	systemctl restart systemd-udevd.service
	# Trigger udev for keyboards already plugged in / built-in
	udevadm trigger --subsystem-match=input --property-match=ID_INPUT_KEYBOARD=1 --action=add

# ── Uninstall ─────────────────────────────────────────────────────────────────

uninstall:
	-systemctl stop 'dvorak@*.service'
	rm -f $(DESTDIR)$(BINDIR)/$(TARGET)
	rm -f /etc/udev/rules.d/80-dvorak.rules
	rm -f /etc/systemd/system/dvorak@.service
	systemctl daemon-reload
	udevadm control --reload
	systemctl restart systemd-udevd.service

# ── Clean ─────────────────────────────────────────────────────────────────────

clean:
	-rm -f $(TARGET) test_dvorak
	cargo clean
