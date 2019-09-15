# ADS1256_DUAL
This is fork of the Flydroid library https://github.com/Flydroid/ADS12xx-Library in order to allow operation of two ADS1256 (24-bit ADC) devices on one Teensy.

I have tested this code running on a Teensy 4 device and two of the yellow ADS1256 boards from eBay that look like this one: https://forum.pjrc.com/threads/49404-TI-ADS1256-8-CH-24-bit-A-D?p=166556&viewfull=1#post166556 and it ran overnight without obvious error with two ADCs running at a 500 Hz sampling rate, so that is about 14 million samples from each ADC.

The original Flydroid library would sometimes give me errors with the original SPI timing. I used a slower SPI clock at 960kHz and also added delays between CS low, the read command, and reading out data.  Also to simplify support for two chips I use a simple busy-wait to poll each DRDY signal, instead of an interrupt.  
