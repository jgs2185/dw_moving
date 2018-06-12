# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_canbus_message_sample CAN Message Interpreter-based on DBC File Sample

The CAN Message Interpreter-based on DBC File sample is a simple CAN bus
interpreter sample. An interpreter is built based on the definition in a DBC
file, and input CAN messages are then decoded by the interpreter. In this
sample, information about car steering and speed is transmitted in CAN messages,
and the sample decodes and displays all received CAN messages. By default, the
sample demonstrates the usage with virtual CAN, using an offline message file.
You may also set the input arguments for real CAN message input (e.g., can0 or
vcan0) on Linux desktop or can.aurix on DrivePX.

#### Usage

Use this sample application with:

- Virtual CAN bus defined in DriveWorks with offline binary CAN message files
- Virtual CAN bus defined by SocketCAN on Linux
- Real CAN device, which supports SocketCAN on Linux


#### Offline CAN Messages

    ./sample_canbus_interpreter

By default, the sample reads `data/samples/sensors/can/sample.dbc` to build the
interpreter, and opens a virtual CAN bus with messages defined in
`data/samples/sensors/can/canbus_dbc.can`.

The output on the console is the car steering and speed. For example:

    5662312708 [0x100] -> 0x9d 0xfe  Car steering -0.0221875 rad at [5662312708]
    5662312708 [0x200] -> 0x1f 0x21  Car speed 8.479 m/s at [5662312708]

#### DBC files

The sample can be used with a DBC file provided by the user. In order to start
interpreter sample using user provided DBC file, pass `--dbc` parameter to the
application:

    ./sample_canbus_interpreter --dbc=user_path_to/file.dbc

#### Virtual CAN Bus on Linux

    ./sample_canbus_interpreter --driver=can.socket --params=device=vcan0

Run it with a virtual CAN device created with:

    $ sudo modprobe vcan
    $ sudo ip link add dev vcan0 type vcan
    $ sudo ip link set up vcan0

To send data from the console to the virtual CAN bus, run `cansend` (from the
`can-utils` package):

    $ cansend vcan0 100#fa490200
    $ cansend vcan0 200#2e920300

This sends a CAN message with the CAN ID and 2 bytes of data, containing: 0x11,
0x22

The output is similar to:

    44798857 [0x100] -> 0xfa 0x49 0x2 0x0  Car steering 0.0937563 rad at [44798857]
    64318678 [0x200] -> 0x2e 0x92 0x3 0x0  Car speed 2.3403 m/s at [64318678]


#### Real CAN Device on Linux

    $ ./sample_canbus_interpreter --driver=can.socket --params=device=can0

A valid SocketCAN device must be present in the system as canX. For example, a
PCAN USB. The bitrate is set to 500 KB. Set the bitrate on the CAN device:

    $ sudo ip link set can0 type can bitrate 500000
    $ sudo ip link set can0 up

The CAN interface used to listen on a real CAN bus must have SocketCAN driver
implementation in the system. A SocketCAN based driver can be identified easily
if `:> sudo ifconfig -a` returns the default network interfaces **and** a CAN
based interface (i.e., canX, slcanX, etc).

##### AurixCAN
On DRIVE PX 2 and ACR, the CAN connectors marked with CAN-1..CAN-4 are reachable throught Aurix by the Tegras.
For this to work, a proper setup of the Aurix needs to be made prior to running the application. Please
refer to the EasyCAN user guide provided as part of PDK documentation to set up Aurix to filter
and pass a selected subset of CAN messages. In order to connect to AurixCAN, use the following arguments:

On DRIVE PX2:
    --driver=can.aurix --params=ip=10.42.0.83,bus=a

On ACR:
    --driver=can.aurix --params=ip=127.0.0.1,aport=50000,bport=50103,bus=a

Where `ip` is the IP address from which the CAN messages are forwarded (by default, set to 10.42.0.83)
and `bus` points to the corresponding CAN bus connector, i.e., CAN-1->a, ..., CAN-4->d.

The connection to AurixCAN happens using UDP sockets. Additional arguments for the remote (`aport`) and
local (`bport`) UDP port can be specified if AurixCAN is working in a non-default configuration. By
default, on DRIVE PX2, AurixCAN is reachable over `aport=50000` and communicates with the Tegra over
`bport=60395`. On ACR however, AurixCAN is reachable over `aport=50000` and communicates with
the Tegra over `bport=50103`.

