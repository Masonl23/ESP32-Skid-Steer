/* Author: Mason Lopez, partially from Rui Santos (https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/)
   Created: 12/15/2021
   Purpose: This code uses 2 ESP32's, one as the controller and the other as the reciever which controls the servos and 
            measures the battery level. In this file this is the reciever which recieved the joystick data from the sender
            or controller ESP, this reciever also sends data to the controller which contains information about the position
            of the arm, bucket and battery voltage. Please note that the battery voltage reader is very crude along with
            much other things in this project and is not very accurate, it is only used to give a general sense of where the
            battery capacity is at anytime.
            Thanks and feel free to contact me or donate
            Mason Lopez - Masonlopez@me.com or Venmo: Masonl6
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include "DifferentialSteering.h"

//mac address of the esp communicating with, use the example given with the esp to find this
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xB2, 0x72, 0x5C};


//struct for recieving control information
typedef struct Control
{
  int right_x;  //right joystick x val
  int right_y;  //right joystick y val
  int left_x;   //left joystick x val
  int left_y;   //left joystick y val
  int left_switch;  //left joystick button
  int right_switch; //right joystick button
} Control;

//struct for reciever to send data back to sender
typedef struct DispStuff
{
  float battery_voltage;
  int arm_angle;
  int bucket_angle;
} DispStuff;

// Pins connected to the ESP
const int arm_pin = 16;
const int bucket_pin = 17;
const int left_wheel_pin = 4;
const int right_wheel_pin = 18;
const int battery_pin = 34;

//variables used for the measuring battery level
float battery_level = 0;  //
float voltage = 0;

// for timing of arm and bucket
unsigned long current_time = 0;
unsigned long last_time = 0;
unsigned long period = 15;
unsigned long last_update = 0;
unsigned long update_period = 60;

// servos
Servo arm;
Servo bucket;
Servo left_wheel;
Servo right_wheel;

// value for the arm and bucket
int arm_val = 90;
int bucket_val = 90;
// value for wheels used by differential steering
int left_wheel_val = 0;
int right_wheel_val = 0;

// accuracy value for the arm and bucket
const int accuracy_val = 3;

// for mg996r servo
const int min_val = 385;
const int max_val = 2450;

// Bucket min and max range
const int bmax = 165;
const int bmin = 0;

// Arm min and max range
const int amax = 138;
const int amin = 25;

DispStuff display_info;
Control current_values;
DifferentialSteering steer;

const int accuracy = 182;
int comp_value = 4095 / (accuracy); // fixes the mapping issue

//Returns the battery voltage approx
float read_voltage(){
  battery_level = analogRead(battery_pin);
  voltage = (((battery_level/4095) * 3) * 2.8) + 0.77;
  return voltage;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{

  Control *controls = (Control *)data;
  //these comp values are very wonky and poor, you will need to most likely edit this value so the value is centered
  int rx = controls->right_x + comp_value;
  int ry = controls->right_y + comp_value;
  int lx = controls->left_x + comp_value;
  int ly = controls->left_y + comp_value;
  int lb = controls->left_switch;
  int rb = controls->right_switch;

  //save the local struct to the incoming data 
  //accuracy value is the precision/increment value the joystick is being mapped to, the greater the value the faster it moves
  current_values.right_x = map(rx, 0, 4095, -accuracy_val, accuracy_val); // bucket
  current_values.right_y = map(ry, 0, 4095, -accuracy_val, accuracy_val); // arm
  current_values.left_x = map(lx, 0, 4095, -127, 128) + 9;                // bucket
  current_values.left_y = map(ly, 0, 4095, -127, 127) + 7;
  current_values.left_switch = lb;
  current_values.right_switch = rb;

  //print the mapped values, useful for debugging and figuring out the comp value mentioned previously
  Serial.print("LEFT  X: ");
  Serial.print(current_values.left_x);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print(current_values.left_y);
  Serial.print("  RIGHT  X: ");
  Serial.print(current_values.right_x);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.println(current_values.right_y);
}

// bool returns true if the value is negative
bool is_negative(int servo_value)
{
  if (servo_value < 0)
  {
    return true;
  }
  return false;
}

// checks whether the joystick has moved
bool is_input(int joystick_input)
{
  if (joystick_input > 0 || joystick_input < 0)
  {
    return true;
  }
  return false;
}

// Checks whether the servo value is in bounds
bool in_bounds(int servo_value)
{
  if ((servo_value) <= 180 && (servo_value >= 0))
  {
    return true;
  }
  return false;
}

// sums the values together and returns the new value
const int new_value(int servo_value, int joystick_input)
{
  return servo_value + joystick_input;
}

// returns the sign of the value
int input_sign(int &servo_value)
{
  if (is_negative(servo_value))
  {
    return -1;
  }
  return 1;
}

//Moves the servo
//joystick_input - the mapped value of the joystick, used as the incrementer
//servo_value - the global variable that holds the value/ angle of the servos position
//input_servo - the servo that is to be incremented/moved
//smin - the min value of the servo
//smax - the max value of the servo
void increment_servo_position(int joystick_input, int &servo_value, Servo &input_servo, int smin, int smax)
{
  if (is_input(joystick_input) && in_bounds(servo_value))
  {

    if (joystick_input < 0)
    {
      if (new_value(servo_value, joystick_input) > smin + 5)
      {
        servo_value += joystick_input;
        input_servo.write(servo_value);
      }
      else if (new_value(servo_value, joystick_input) <= (smin + 5) && ((servo_value - 1) >= smin)) //makes the servo move slowly near endstops
      {
        servo_value -= 1;
        input_servo.write(servo_value);
      }
    }

    else if (joystick_input >= 0)
    {
      // Serial.println("positive");
      if (new_value(servo_value, joystick_input) < smax - 5)
      {
        servo_value += joystick_input;
        input_servo.write(servo_value);
      }
      else if (new_value(servo_value, joystick_input) >= (smax - 5) && ((servo_value + 1) <= smax)) //makes the servo move slowly near endstops
      {
        servo_value += 1;
        input_servo.write(servo_value);
      }
    }
  }
}
void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_peer_info_t peerInfo;
  esp_now_register_recv_cb(OnDataRecv); //register the recieve data function that will be used
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  //attach the arm pin along with its max and min pwm signals
  arm.attach(arm_pin, min_val, max_val);
  arm_val = arm.read(); //read the position of the arm so it knows relatives position
  //attach the bucket pin along with its max and min pwm signals
  bucket.attach(bucket_pin, min_val, max_val);
  bucket_val = bucket.read();
  //attach the left and right wheel servos (left wheel is connected to 2 servos and right wheel is connected to 2 servos)
  left_wheel.attach(left_wheel_pin);
  right_wheel.attach(right_wheel_pin);
  disableCore0WDT(); // weird bug that causes crashes if not disabled for me
  //allocate the timers for the PWM which moves the servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  //the period of the pwm for the arm
  arm.setPeriodHertz(50);
  //the differential steering library, believe it controls how fast it can spin on its axis, lower the better 
  steer.begin(2);
  //declare the battery pin is an input so it can read the analog value
  pinMode(battery_pin, INPUT);  //NOTE THAT THE 7.4 BATTERY IS CONFIGURED IN A VOLTAGE DIVIDER OF 18K AND 10K SUCH THAT THE
                                //MAX VOLTAGE THE ESP CAN RECIEVE IS 3 VOLTS (CONSIDERING 8.4V AS MAX INPUT), FAILURE TO DO 
                                //THIS CAN RESULT IN DAMAGE TO THE BOARD!!
}

void loop()
{
  current_time = millis();

  //Check if the period alloted has expired
  if (current_time - last_time >= period)
  {
    increment_servo_position(current_values.right_y, arm_val, arm, amin, amax);
    increment_servo_position(current_values.right_x, bucket_val, bucket, bmin, bmax);
    if (current_values.left_switch == LOW)  //shortcuts when left joystick is pressed the bucket lowers to scooping pos.
    {
      bucket.write(103);
      arm.write(amin);
      bucket_val = 103;
      arm_val = amin;
    }
    if (current_values.right_switch == LOW) //shorcut when right joystick is pressed the bucket raises 
    {
      bucket.write(103);
      arm.write(amax);
      bucket_val = 103;
      arm_val = amax;
    }
    last_time = millis();
  }
  //check if the update period has passwed, this could be put into one such as above but i wanted to make
  //the voltage read more able to read, the longer period makes the voltage reading more stable
  if (current_time - last_update >= update_period){
    display_info.battery_voltage = read_voltage();
    display_info.arm_angle = arm_val;
    display_info.bucket_angle = bucket_val;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&display_info, sizeof(DispStuff));
    last_update = millis();
  }

  steer.computeMotors(current_values.left_x, current_values.left_y);  //differential steering library put the mapped values into
  left_wheel_val = steer.computedLeftMotor();
  right_wheel_val = steer.computedRightMotor();
  left_wheel_val = map(left_wheel_val, -127, 127, 0, 180);  //convert the values from the joystick to servo readable
  right_wheel_val = map(right_wheel_val, -127, 127, 0, 180);
  //note that the servos i am using for driving are continous rotation (mg90r) and they accpet 0-180
  left_wheel.write(left_wheel_val);
  right_wheel.write(180 - right_wheel_val); //the servos are facing opposite ways so subtract 180 to get normal movement
} 