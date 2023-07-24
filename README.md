## M5Stamp S3 ESP-IDF VLF reciever

This is an experimental application of M5Stamp S3 using I2S audio ADC AK5720VT.
It samples 0-48kHz signals with 96k sampling rate and sends raw|fft data with UDP/WiFi.
The default UDP target address and port can be configurable at build time. The address can be modified with sending command to TCP port.

```
telnet inet_address_of_reciever 5990
...
help
Command list:
 mode [fft|raw]
 addr aa.bb.cc.dd
 help
```

[UDP packet]
```
raw mode: 1024 samples 2ch S32_LE
fft mode: 512  samples IQ for L signal S32_LE 
```
[Build]

Set normal ESP-IDF build environment with

```
source ESP_IDF_PATH/export.sh
idf.py menuconfig
```

and add the led_strip library with

```
idf.py add-dependency "espressif/led_strip^2.4.1"
```

then run "idf.py build".

[schematics]

![schematic of module](https://github.com/kazkojima/vlfstamp/blob/main/images/vlfstamp-sch.png)
