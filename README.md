# Virtual Serial Port Kernel Module

## Introduction

This module creates virtual serialport pairs that can be used for communication between user space applications.

### Features:

- [x] Support for multiple virtual port pairs
- [x] Support setting different baudrates and data formats(drops data silently if not matched)
- [x] a simple program for user-space management
- [ ] soft/hard flow control
- [ ] modem ioctl

## Installation

### Manual Installation

1. Clone this repository

2. Add user to uucp/dialout group:

   ```bash
   #Arch linux
   sudo usermod -aG uucp $USER
   #Debian/Ubuntu
   vim 99-vserial.rules #change uucp to dialout
   sudo usermod -aG dialout $USER
   ```

3. Build and install module:

   ```bash
   sudo make modules_install
   ```

4. Load module:

   ```bash
   sudo modprobe vserial
   ```

5. Compile vserialctl (for user-space management)

   ```bash
   gcc -o vserialctl vserialctl.c
   ```

6. Add virtual port pairs using vserialctl:

   ```bash
   ./vserialctl -s 1
   ```

7. Test

   ```bash
   #115200 8N1 by default
   cat /dev/ttySV1
   #run in a new terminal
   echo "Hello, World!" > /dev/ttySV0
   ```

## Install from AUR (Arch linux)

   ```bash
   # this will also add vserialctl to PATH
   paru -S vserial
   sudo systemctl enable vserial --now
   ```
