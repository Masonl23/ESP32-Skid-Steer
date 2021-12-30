# ESP32-Skid-Steer
Bruder Catepillar Skid Steer model converted to RC, controlled by an ESP32 with 2 analog joysticks and a receiver that is an ESP32 on the model.

![4B1A9B2F-0BB2-496A-850F-82EEA048991A_1_105_c](https://user-images.githubusercontent.com/30536263/147759488-4af25e25-ace5-4c25-9380-d2e993a4cfb4.jpeg)




> Written with [StackEdit](https://stackedit.io/).
> # ESP32 Skid Steer Project
> In this project I converted a [Bruder Skid Steer](https://www.amazon.com/bruder-02482-Caterpillar-Steer-Loader/dp/B07P1673W3/ref=sr_1_3?crid=2AN2IPYR6IWEQ&keywords=bruder%20skid%20steer&qid=1640876858&sprefix=bruder%20skid%20steer,aps,85&sr=8-3) model into a remote controlled version controlled by 2 analog joysticks, of which is then transmitted by an ESP32 to another ESP32 via ESPNOW protocol. The receiver ESP controls the servos with PWM after mapping the raw data from the sender ESP32.


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

# Libraries Used 

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
 
 
