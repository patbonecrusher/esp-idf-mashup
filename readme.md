# esp-idf: breathing led using c++

## Rationals

The goal of this experiment was to:  

- Port [the arduino breath/blink led](https@github.com:patbonecrusher/esp32-breathy.git) to the esp-idf framework
- Use esp-idf task
- Objectifying task creation in FreeRTOS

### The port

I couldn’t figure out how to get my led to breathe as smoothly as it did with the Arduino framework.
To figure it out I:

- Cloned and build the [Arduino framework](https://github.com/espressif/esp32-arduino-lib-builder )
- Reversed engineer how the Arduino framework led channel is implemented
- Created my own LedChannel class based on the Arduino C implementation
- __note__ the class is a very poor attempt at doing so.  I just ported the bare minimum for my app to work.

### Objecting task creation in FreeRTOS

I simply followed the instructions from that wonderful post:
[Objectifying task creation in FreeRTOS – Electrónica y Sistemas Embebidos](https://fjrg76.wordpress.com/2018/05/20/objectifying-task-creation-in-freertos/)
[OBJECTIFYING TASK CREATION IN FREERTOS (II) – Electrónica y Sistemas Embebidos](https://fjrg76.wordpress.com/2018/05/23/objectifying-task-creation-in-freertos-ii/)

## Instructions

Clone this repo

```bash
# Follow the idf.py instructions
idf.py build
idf.py flash
idf.py monitor
```
