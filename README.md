### FXIMU_firmware

**Notice: If you are just going to use the FXIMU, you dont need this repository**  

**Project main repository: https://github.com/altineller/fximu**

This repository contains the FXIMU firmware code, in case you want to do changes in the firmware for your application. You will need a XDS110 debugger to upload to FXIMU.

### Prerequisites

**Install the following packages:**

```
sudo apt install libgconf-2-4 libncurses5 libpython2.7 libtinfo5 -y
sudo apt install lm4flash -y
sudo apt install gcc-arm-none-eabi -y
```


**Download SW-TM4C: TivaWare for C Series Software (Complete) from TI's website.**

Extract it to directory of your choosing. Example:

```
mkdir ~/ti
cd ~/ti
mv <directory_downloaded>/SW-TM4C-2.2.0.295.exe ~/ti
unzip SW-TM4C-2.1.1.71.exe
rm SW-TM4C-2.1.1.71.exe
```

Add the following to your `~/.bashrc` file:

```
export TIVA_WARE_PATH=/home/ubuntu/ti/SW-TM4C-2.2.0.295
export TIVA_FLASH_EXECUTABLE=lm4flash
```

**Download rosserial_tivac and build**

>Notice: There are more than one version of `rosserial_tivac`  
>Use `rosserial_tivac directory` from https://github.com/ros-drivers/rosserial


```
git clone https://github.com/ros-drivers/rosserial.git
cd rosserial
mv rosserial_tivac ~/catkin_fx/src
```

You can then remove the rosserial directory that you cloned from git.

To see if it works:		

```
cd catkin_fx
catkin_make
```

It should make without any errors.

### Compiling Firmware

```
cd ~/catkin_fx
catkin_make fximu_firmware_complementary_complementary.axf
```

### Updating Firmware

You either need the XDS110 debug probe, or TI TivaC development board and a JTAG adapter  to update the firmware.

Connect your programmer to JTAG port on the FXIMU. The little arrow indicates pin number  one.

```
cd ~/catkin_fx
catkin_make fximu_firmware_complementary_flash
```

You should see it writing and verifying firmware to the device.

For the XDS110 debug proble you need a custom upload script provided by CCS. For using the TivaC as a JTAG programmer, you need `lm4_flash` which is installed in the prerequisites.


### Tivaware USB library bug

In the file: `usblib/usbbuffer.c`

```
USBBufferInfoGet(const tUSBBuffer *psBuffer, tUSBRingBufObject *psRingBuf)
    (...)
        psRingBuf->ui32Size = psBufVars->sRingBuf.ui32ReadIndex;
```

Change the last line above with:

        psRingBuf->ui32Size = psBufVars->sRingBuf.ui32Size;

(c) Copyright, 2019, Can Altineller altineller@gmail.com

Free for non commercial use
