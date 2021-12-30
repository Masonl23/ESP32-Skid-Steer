/* Author: Mason Lopez, partially from Rui Santos (https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/)
   Created: 12/15/2021
   Purpose: This code uses 2 ESP32's, one as the controller and the other as the reciever which controls the servos and 
            measures the battery level. In this file this is the transmitter or sender which communicates to the ESP 
            connected to the skid steer. This sender also recieves information from the reciever about the current arm,
            bucket and battery level of the reciever. This sender sends the joystick inputs. Please note that this code 
            is in no way done the most efficient way, my coding knowledge is limited to what is shown.
            Thanks and feel free to contact me or donate
            Mason Lopez - Masonlopez@me.com or Venmo: Masonl6
*/


#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>

// Mac address of ESP32 communicating with
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xB2, 0x64, 0xEC};

// pins for the left and right joysticks
const int right_joy_x = 36;
const int right_joy_y = 39;
const int left_joy_x = 35;
const int left_joy_y = 34;
const int right_button = 23;
const int left_button = 22;

// battery voltage
float bat_voltage = 0;

// struct for the joystick
typedef struct Control
{
  int right_x;
  int right_y;
  int left_x;
  int left_y;
  int left_switch;
  int right_switch;
} Control;

//create instantiation
Control controls;

// struct for reciever to send data back to sender
typedef struct DispStuff
{
  float battery_voltage;
  int arm_angle;
  int bucket_angle;
} DispStuff;

// time control for sending the new variables
unsigned long current_time = 0;
unsigned long last_time = 0;
unsigned long period = 15; // every 5ms there will be update to reciever

// for display
int arm_val = 0;
int bucket_val = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 'skidsteerclipart-4', 60x35px
const unsigned char epd_bitmap_skidsteerclipart_4[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf2, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x12, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x13, 0xe2, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x1f, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x70, 0x86, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x37, 0x87, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x3a, 0x8a, 0x00, 0x00,
    0x00, 0x00, 0x01, 0xe1, 0xfb, 0x8a, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0f, 0xf1, 0x8a, 0x00, 0x00,
    0x00, 0x00, 0x04, 0xf0, 0x00, 0x8a, 0x00, 0x00, 0x00, 0x00, 0x05, 0xf3, 0xff, 0x8a, 0x00, 0x00,
    0x00, 0x00, 0x05, 0x30, 0x02, 0x8a, 0x00, 0x00, 0x00, 0x00, 0x19, 0xfc, 0x0f, 0xca, 0x00, 0x00,
    0x00, 0x00, 0x2b, 0x06, 0x10, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x7b, 0x3a, 0x23, 0x2a, 0x00, 0x00,
    0x00, 0x00, 0x92, 0x0b, 0x28, 0xbc, 0x00, 0x00, 0x00, 0x01, 0x0e, 0x89, 0xe8, 0xb8, 0x00, 0x00,
    0x00, 0x02, 0x0e, 0xcb, 0x6d, 0xa0, 0x00, 0x00, 0x00, 0x07, 0xed, 0x72, 0x37, 0x20, 0x00, 0x00,
    0x00, 0x07, 0xf9, 0x84, 0x18, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 304)
const int epd_bitmap_allArray_LEN = 1;
const unsigned char *epd_bitmap_allArray[1] = {
    epd_bitmap_skidsteerclipart_4};

void updateDisplay(int minArm, int minBucket, int maxArm, int maxBucket)
{
  display.clearDisplay();
  display.drawBitmap(80, 0, epd_bitmap_skidsteerclipart_4, 60, 35, WHITE);
  display.setTextColor(WHITE);
  display.setFont(NULL);
  display.setCursor(28, 0);
  display.setTextSize(1);
  display.print("Arm");
  display.setCursor(52, 0);
  display.print("Bucket");
  display.setCursor(0, 12);
  display.print("Now:");
  display.print(arm_val);
  display.setCursor(54, 12);
  display.print(bucket_val);
  display.setCursor(0, 26);
  display.print("Min:");
  display.print(minArm);
  display.setCursor(54, 26);
  display.print(minBucket);
  display.setCursor(0, 40);
  display.print("Max:");
  display.print(maxArm);
  display.setCursor(96, 40);
  display.print("Volt:");
  display.setCursor(54, 40);
  display.print(maxBucket);
  display.drawLine(28, 8, 44, 8, 1);
  display.drawLine(52, 8, 86, 8, 1);
  display.setCursor(0, 54);
  display.print("Uptime: ");
  display.print(millis() / 1000);
  display.setCursor(96, 54);
  display.print(bat_voltage);
  display.display();
}

//when the ESP32 recieves data it will perform this function, saving the incoming information
//to the local variables that will be used for the oled display
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
  DispStuff *dispStuff = (DispStuff *)data;
  arm_val = dispStuff->arm_angle;
  bucket_val = dispStuff->bucket_angle;
  bat_voltage = dispStuff->battery_voltage;
}

void setup()
{

  Serial.begin(115200);
  
  //set joysticks as inputs
  pinMode(right_joy_x, INPUT);
  pinMode(right_joy_y, INPUT);

  //pullup for the button pins
  pinMode(left_button, INPUT_PULLUP);
  pinMode(right_button, INPUT_PULLUP);  

  //change the sda and sck pins for oled
  Wire.begin(17, 16);
  WiFi.mode(WIFI_STA);

  //start espnow communication
  if (esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //set the function that will execute when data is recieved 
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //if the display does not intialize then cause an error
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.clearDisplay();
  // load the skidsteer animation 
  for (int i = 0; i < 90; i += 10){
    display.drawBitmap(i, 0, epd_bitmap_skidsteerclipart_4, 60, 35, WHITE);
    display.display();
    delay(300);
    display.clearDisplay();
  }
}

void loop()
{
  current_time = millis(); // set the current time
  if (current_time - last_time >= period) //if the last time checked was greater than period
  {
    //update the struct variables by reading pins
    controls.left_x = analogRead(left_joy_x);
    controls.left_y = analogRead(left_joy_y);
    controls.right_x = analogRead(right_joy_x);
    controls.right_y = analogRead(right_joy_y);
    controls.left_switch = digitalRead(left_button);
    controls.right_switch = digitalRead(right_button);
    
    //send the result and save status to variable
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&controls, sizeof(Control));
    if (result == ESP_OK) { //check the status
      Serial.println("Sent with success");
    }
    else{
      Serial.println("Error sending the data");
    }
    updateDisplay(25, 0, 138, 165); //update the oled with new information
    last_time = millis(); //save as last time checked
  }
}