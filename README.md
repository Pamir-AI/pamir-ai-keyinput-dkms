# Pamir AI Key Input DKMS Module

The Pamir AI Key Input DKMS module provides kernel-level support for button inputs from an RP2040 microcontroller over UART communication for Raspberry Pi CM5 systems.

## Overview

This package contains the DKMS (Dynamic Kernel Module Support) module for the Pamir AI key input driver. The driver enables communication with an RP2040 microcontroller to handle button inputs via UART interface.

## Features

- **Button Support**: UP, DOWN, SELECT, and POWER buttons
- **Configurable Debouncing**: Adjustable debounce timing to prevent false triggers
- **Dual Protocol Support**: 
  - Text-based protocol (decimal values)
  - Raw binary protocol for efficiency
- **Bidirectional Communication**: Character device interface for UART transmission
- **Standard Input Integration**: Uses Linux input subsystem for button events
- **Device Tree Configuration**: Easy configuration through device tree overlays

## Hardware Requirements

- Raspberry Pi CM5 or compatible system
- RP2040 microcontroller connected via UART
- UART2 interface (GPIO 4/5) configuration

## Installation

The DKMS module is automatically installed when you install the `pamir-ai-keyinput-dkms` package:

```bash
sudo apt install pamir-ai-keyinput-dkms
```

## Configuration

### Device Tree Overlay

The module uses a device tree overlay to configure the UART interface and driver parameters:

```
dtoverlay=pamir-ai-keyinput
```

### Available Parameters

You can customize the behavior using device tree parameters:

- `debounce=<ms>`: Set debounce time in milliseconds (default: 50)
- `raw_protocol=on/off`: Enable raw binary protocol (default: on)
- `report_press_only=on/off`: Only report key press events (default: off)
- `recovery_timeout=<ms>`: UART recovery timeout (default: 1000)

Example with parameters:
```
dtoverlay=pamir-ai-keyinput,debounce=100,raw_protocol=off
```

## Usage

### Input Events

The driver creates a standard Linux input device that generates key events:

- **UP Button**: `KEY_UP`
- **DOWN Button**: `KEY_DOWN`
- **SELECT Button**: `KEY_ENTER`
- **POWER Button**: Currently not reported

### Character Device

The module also creates a character device `/dev/pamir-keyinput` for bidirectional UART communication:

```bash
# Send data to the RP2040
echo "command" > /dev/pamir-keyinput

# Read data in applications
cat /dev/pamir-keyinput
```

### Testing

Test the input functionality:

```bash
# List input devices
cat /proc/bus/input/devices | grep -A 10 "Pamir AI Key Input"

# Test button events
evtest
# Then select the Pamir AI Key Input device

# Monitor input events
sudo inputattach --daemon /dev/ttyAMA2
```

## Protocol

### Text Protocol (Default Off)

When `raw_protocol` is disabled, the driver expects decimal values followed by newlines:

```
0\n    # No buttons pressed
1\n    # UP button pressed
2\n    # DOWN button pressed
4\n    # SELECT button pressed
8\n    # POWER button pressed
3\n    # UP + DOWN buttons pressed
```

### Raw Protocol (Default On)

When `raw_protocol` is enabled, each byte represents button state:

- Bit 0: UP button
- Bit 1: DOWN button
- Bit 2: SELECT button
- Bit 3: POWER button

## Troubleshooting

### Module Not Loading

```bash
# Check if module is loaded
lsmod | grep pamir-ai-keyinput

# Load module manually
sudo modprobe pamir-ai-keyinput

# Check kernel messages
dmesg | grep pamir
```

### UART Communication Issues

```bash
# Check UART configuration
sudo raspi-config
# Enable UART in Interface Options

# Check device tree overlay
ls -la /boot/firmware/overlays/pamir-ai-keyinput.dtbo

# Verify boot configuration
grep "dtoverlay=pamir-ai-keyinput" /boot/firmware/config.txt
```

### Input Device Not Found

```bash
# List all input devices
ls -la /dev/input/

# Check input device registration
cat /proc/bus/input/devices
```

### Character Device Issues

```bash
# Check character device
ls -la /dev/pamir-keyinput

# Check device permissions
sudo chmod 666 /dev/pamir-keyinput
```

## Manual Build

If automatic DKMS build fails:

```bash
# Build manually
sudo dkms build pamir-ai-keyinput/1.0.0

# Install manually
sudo dkms install pamir-ai-keyinput/1.0.0

# Remove if needed
sudo dkms remove pamir-ai-keyinput/1.0.0 --all
```

## Development

### Source Code

The driver source code is located at:
```
/usr/src/pamir-ai-keyinput-1.0.0/
```

### Key Files

- `pamir-ai-keyinput-main.c`: Main driver implementation
- `pamir-ai-keyinput-overlay.dts`: Device tree overlay source
- `Makefile`: Build configuration
- `dkms.conf`: DKMS configuration

## Support

For technical support and bug reports:

- Website: https://www.pamir.ai
- Email: support@pamir.ai

## License

This driver is licensed under the GPL v2 license. See the source code for full license details.

## Changelog

### Version 1.0.0
- Initial release
- Basic button input support
- UART bidirectional communication
- Device tree overlay support
- DKMS integration 
