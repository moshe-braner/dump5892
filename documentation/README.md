# DUMP5892 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [![Join the chat at https://gitter.im/SoftRF-open/community](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/SoftRF-open/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Software for DIY 1090ES ADS-B receiver based on the GNS 5892 module

## Background

DIY receivers for 1090ES ADS-B messages have settled on SDR methods.  The receiver is broadband and the software looks for certain types of signals within the huge amount of raw data from the SDR module.  This processing of the data requires significant processing power.  So much so that the typical hardware (Raspberry Pi) needs a fan for cooling.  This is not a good approach for portable devices that need to run on a battery for many hours.

Modules do exist for receiving and processing 1090 MHz transponder data specifically.  One is the GNS 5892.  It is small, low-power (1/8 of a watt), and relatively inexpensive.  This project focuses on creating software to handle the data stream from a GNS 5892.

The goal is to integrate these hardware and software modules into [SoftRF](https://github.com/moshe-braner/SoftRF) 
running on a "T-Beam" board with an ESP32 processor.  The processor is already pretty busy doing everything else in SoftRF.  Thus the addition of the ADS-B receiving functions needs to be done in as efficient a manner as possible.  All aspects of the data processing in dump5892 are therefore optimized as best as possible.  Values are pre-computed, lookup tables prepared, integer math is used instead of floating point where possible, etc.  Also the software has the ability to filter out traffic that is "too far" or "too high" etc early in the processing.

The data processing has been wrapped in additional software that turns it into a standalone "app" that can also be used to explore the local air traffic.

<br>

![](https://github.com/moshe-braner/dump5892/tree/master/documentation/GNS5892_and_ESP32mini_.jpg)

<br>

## Overview

The target hardware is the GNS5892 module and a generic ESP32 board.  They should be connected with short wires, as the output baud rate of the GNS5892 is 921600 baud.  The 3.3V power output from a typical ESP32 board should be able to supply the GNS5892 the 40 mA it requires.

The USB jack of the ESP32 board should then be connected to a computer on which a USB terminal program is running at 115200 baud.  It serves as both the user interface for commands and operational info, as well as the data collection tool, by letting it run for a while and then saving the text in the terminal program.  Other software could theoretically be written to handle the computer side of things in friendlier ways.

The "app" running on the ESP32 has various settings and modes.  Type "?" into the terminal to see the list of commands.  ADS-B messages in the format output by the GNS5892 can also be entered into the terminal and will be processed in the same way as messages coming from the GNS5892.  This is useful for debugging.

Due to the focus on preparing software for situational awareness in a portable device carried in an aircraft, only the "relative" mode of position decoding is included.  One must enter a reference location in order to decode the locations of aircraft from which messages are received.  This location does not have to be precise, it can be within tens of miles of the traffic.  But if it is correct for a specific point, whether the receiver location or a nearby airport, dump5892 can report the traffic locations as distance and bearing from the reference location, instead of lat/lon coordinates.

The source code is set up to compile for ESP32 under the Arduino IDE.


## Links

Open [discussion forum](https://gitter.im/SoftRF-open/community).
<br>

[Source code](https://github.com/moshe-braner/dump5892/tree/master/source/dump5892)
<br>

[Compiled binaries for ESP32](https://github.com/moshe-braner/dump5892/tree/master/binaries)
<br>


## Version history

### revision 07

Added options.  UI improvements.  Fixed minor bugs.

### revision 06

Cleaned up distance, bearing, speed and track calculations to use appropriate approximation for each.  Fixed minor bugs.

### revision 05

Revised output flow control, to work with Arduino ESP32 Core 2.0.3 in which Serial.availableForWrite() is broken.

### revision 04

Added decoding of Mode S altitude replies (DF4,20).
Added more values in message-type field.
UI bug fixes and improvements.

### revision 03

Added message-type field (I/P/V) to output.
Made TXT output columns consistent.
Removed spaces from output sentences in TAB & CSV formats.
Fixed some UI bugs.

### revision 02

Completed handling of southern-hemisphere latitudes.
Added raw-but-filtered output option.

### revision 01

Initial version.