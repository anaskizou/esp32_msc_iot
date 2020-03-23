### Esp 32 Embedded Systems school project

<br /><br />
This a school related project and thus is maintained by  SFA Poitiers students.<br />
The goal of this project is to use ESP 32 microcontroller and some peripherals including an RTC and sensors.<br />

The equipements :<br />
Esp-wrover-kit V4.1 that includes and esp32, ili9341 screen and JTAG more info : [Official espressif](https://docs.espressif.com/projects/esp-idf/en/latest/hw-reference/get-started-wrover-kit.html#get-started-esp-wrover-kit-v4-1-board-front "Espressif docs link")<br />

In order to use this project :<br />

1. Clone the project :

```bash
git clone --recursive https://github.com/anaskizou/esp32_msc_iot.git 
```
2. Build the project :

```bash
idf.py build 
```
3. Check the configuration if necessary :

```bash
idf.py menuconfig 
```
4. Build and flash the program :

```bash
idf.py -p /dev/ttyUSB1 flash monitor 
```

*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*
