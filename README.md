### FXIMU_firmware

**Notice: You dont need this repository, if you are just going to use the FXIMU**

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
