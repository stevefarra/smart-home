# Smart Home IoT Device
This repository contains the code for the STM32F407VGT used in the IoT device, which is responsible for reading and debouncing
pushbuttons, then relaying information between itself and an ESP8266 Wi-Fi chip (which provides connection to a mobile app)
as well as an RF module. Below is an explanation of the communication protocols used between each device.

## STM32F4 → ESP8266
Every second, the status of each button is transmitted through UART. 1 byte is sent for each button. The LSB indicates the 
button status (0 for `OFF`, 1 for `ON`), and the remaining 7 bits represent incidate the button ID, with 0 being the first 
button. For instance, `00000101` indicates that the 3rd button is on.

## STM32F4 → RF Module
Whenever a button is toggled, a packet is sent through radio using a GPIO pin. A radio packet is 19 bits, with the least 
significant 3 bits indicating the status of the button (0 for `OFF`, 1 for `ON`) and the remaining bits representing the button 
ID, with 0 being the first button. For instance, `0000000000000011000` indicates that the 4th button has been turned off. 
Each radio packet is sent 6 times, with a 1000 µs delay between each one, for data integrity purposes. To transmit a 1, the 
pin is set high for 500 µs and then low for 500 µs. To transmit a 0, the pin is set high for 250 µs and low for 250 µs.

## ESP8266 → STM32F4
For changes made to the buttons by external devices, the STM32 also receives packets via UART with the same format that they
are sent in. The UART packets that are sent back are changed accordingly and a radio packet is also sent to account for the 
change.
