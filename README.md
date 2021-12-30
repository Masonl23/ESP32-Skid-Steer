# ESP32-Skid-Steer
Bruder Catepillar Skid Steer model converted to RC, controlled by an ESP32 with 2 analog joysticks and a receiver that is an ESP32 on the model.

![4B1A9B2F-0BB2-496A-850F-82EEA048991A_1_105_c](https://user-images.githubusercontent.com/30536263/147759488-4af25e25-ace5-4c25-9380-d2e993a4cfb4.jpeg)










> Written with [StackEdit](https://stackedit.io/).
> # ESP32 Skid Steer Project
> In this project I converted a [Bruder Skid Steer](https://www.amazon.com/bruder-02482-Caterpillar-Steer-Loader/dp/B07P1673W3/ref=sr_1_3?crid=2AN2IPYR6IWEQ&keywords=bruder%20skid%20steer&qid=1640876858&sprefix=bruder%20skid%20steer,aps,85&sr=8-3) model into a remote controlled version controlled by 2 analog joysticks, of which is then transmitted by an ESP32 to another ESP32 via ESPNOW protocol. The receiver ESP controls the servos with PWM after mapping the raw data from the sender ESP32.
> 
 [![ESP32 Bruder](http://img.youtube.com/vi/Ya_37AnNs0E/0.jpg)](http://www.youtube.com/watch?v=Ya_37AnNs0E "ESP32 remote skid steer")

[![Bruder ESP32](http://img.youtube.com/vi/nQAw4l_MV88/0.jpg)](http://www.youtube.com/watch?v=nQAw4l_MV88 "Video Title")
>


#  ⚙️ Parts used
Note that these parts below are the ones that i used in my build, you do not need to use the exact same servos. If you are not in a time crunch I recommend buying some of these parts from [aliexpress.com](aliexpress.com), you can save a little bit of money in exchange for longer delivery time.
 - [Bruder Skid Steer](https://www.amazon.com/bruder-02482-Caterpillar-Steer-Loader/dp/B07P1673W3/ref=sr_1_3?crid=2AN2IPYR6IWEQ&keywords=bruder%20skid%20steer&qid=1640876858&sprefix=bruder%20skid%20steer,aps,85&sr=8-3) 
 - 2x [ESP32 dev modules](https://www.amazon.com/KeeYees-Development-Bluetooth-Microcontroller-ESP-WROOM-32/dp/B07QCP2451/ref=sr_1_5?crid=27PPD7MB3HTA8&keywords=esp32%20development%20board&qid=1640877264&sprefix=esp32%20,aps,97&sr=8-5)  The sender and receiver modules
 - 1 high torque servo like [KS-3518](https://www.amazon.com/HONG-YI-HAT-KS-3518-Digital-Waterproof/dp/B08961JWG7), [MG996r](https://www.amazon.com/4-Pack-MG996R-Torque-Digital-Helicopter/dp/B07MFK266B/ref=sr_1_5?keywords=mg996r&qid=1640877491&sr=8-5) or any equivalent strength for moving the arm
 - 1 [MG90s](https://www.amazon.com/MG90S-Servo-Motor-Helicopter-Arduino/dp/B07L6FZVT1/ref=sr_1_5?crid=1M1O5G0CVSXWE&keywords=mg90s&qid=1640877631&sprefix=mg90s,aps,87&sr=8-5) servo for controlling the bucket 
 - 4 [MG90s ](https://www.amazon.com/Compatible-Raspberry-Project-Helicopter-Airplane/dp/B0925TDT2D/ref=sr_1_6?crid=1M1O5G0CVSXWE&keywords=mg90s&qid=1640877736&sprefix=mg90s,aps,87&sr=8-6) continuous moving for driving the wheels, note you may want to pick better quality servos as these have a lot of slop in them, which is why i used a washer in between the wheel and servo as you'll see later
 - [OLED SSD1306 i2c dislpay 128x64](https://www.amazon.com/DIYmall-Serial-128x64-Display-Arduino/dp/B00O2KDQBE/ref=sr_1_5?crid=2UMH0Y2BOUL5X&keywords=oled%20ssd1306&qid=1640879380&sprefix=oled%20ss,aps,93&sr=8-5)
 - 2 [Joystick](https://www.amazon.com/DEVMO-Joystick-Breakout-Controller-Arduino/dp/B07R7736QH/ref=sr_1_10?crid=SLVVSCZ3MQYJ&keywords=arduino%20analog%20joystick&qid=1640879626&sprefix=arduino%20anaog%20jo,aps,86&sr=8-10) modules 
 - 18k and 10k resistor and optional capacitors (I used 47uF but anything really works)
 - Buck converter 7.4v to 5v, any brand works however i realized later on that this is not necessary as the ESP32 can handle 7.4v (on vin pin)! However i am still using it .
 - (optional) Y servo connector, connects the two servos on each side together, you could optionally just solder them together
 - Perforate DIY breadboard for easy soldering 
 - Will need access to a 3D printer for certain parts

# 📚Libraries Used 

    #include  <ESP32Servo.h>
    #include  <esp_now.h>
    #include  <WiFi.h>
    #include  "DifferentialSteering.h"
    #include  <Wire.h>
    #include  <Adafruit_GFX.h>
    #include  <Adafruit_SSD1306.h>
    #include  <Adafruit_I2CDevice.h>

 - [ESP32Servo](https://www.arduino.cc/reference/en/libraries/esp32servo/) library
 - [DifferentialSteering](https://github.com/edumardo/DifferentialSteering) library
 The rest you should be able to download from the library manager on Arduino
 
# 🛠 How to build 
Watch this [video](https://www.youtube.com/watch?v=ahsBrwnNrWI&), and this [one](https://www.youtube.com/watch?v=ji3Fsj78cEw&). These videos provide a great help and converting the model into the RC version, I followed everything in that video except for the motors for the wheels, converting the servo into continuous for me and the creator resulted in random jitter as it was very difficult to center the internal potentiometer. 

The 3D files I [used](https://www.thingiverse.com/thing:1989001)  which was for the servo bucket attachment, i did not use the other parts as i messed up my wheels and used a different servo for the arm.

The 3D files I [created](https://www.thingiverse.com/thing:5179237) the washer to fix the wheel wobble, inner wheel hub, and the tool used to make the hole circular in the rubber tire. The tool is meant to be wrapped with sandpaper and put into a drill so that making the hole circular is easy.

**For connections**
|Receiver ESP32 |Sender ESP32  |
|--|--|
|Pin 16 will be used for arm PWM | Pin 36 connected to the right x joystick|
Pin 17 will be used for bucket PWM|Pin 39 connected to the right y joystick|
Pin 4 will be used for the left servos PWM|Pin 35 connected to the left x joystick|
Pin 18 will be used for the right servos PWM|Pin 34 connected to the left y joystick|
Pin 34 will be used for the reading analog battery level|Pin 23 connected to the right joystick button |
||Pin 22 connected to the left joystick button|
||Pin 17 to SDA of OLED|
||Pin 16 to SCK of OLED|
For the Receiver ESP32 it will be easier if you create a row of 3 long header pins that make it easier to connect servos to.

In addition to this you may find it useful to solder in a capacitor between the 5v and ground connection of both receiver and sender.

I am omitting the instructions to incorporate the motors into the model due to the fact that the videos mentioned above do a great job explaining.

# 🖥 Code

To use the provided code you must first use the following sketch to find the Mac Address of each device

    
```c
// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/

#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}
```
Upload this code and both devices and write down each of the Mac Addresses, into the respective copy and replace these addresses in the [sender](https://github.com/Masonl23/ESP32-Skid-Steer/tree/main/sender_esp) and [receiver](https://github.com/Masonl23/ESP32-Skid-Steer/tree/main/reciever_ESP) files above.

Once you have replaced the following you are able to upload the code to each, once uploaded you are most likely going to need to fix the mapping of the joysticks, i tried using math to add a value to the map to fix the issue known as `comp_value` but this is very poorly done.

# Thanks!
That is all for this project, i would like to give special thanks to Rui Santos who has the code on his [website](https://randomnerdtutorials.com/get-change-esp32-esp8266-mac-address-arduino/)  Random Nerd Tutorials, this site helped a lot! 

Thanks,

Mason Lopez 
Masonlopez@me.com
