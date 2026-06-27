/*
        DIY Arduino based RC Transmitter
  by Dejan Nedelkovski, www.HowToMechatronics.com
  Also thanks for ELECTRONOOBS and Max Imagination
  Modified by: Smith Gabijan
  Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>

// Define the digital inputs
#define jB1 1  // Joystick button 1
#define jB2 0  // Joystick button 2
#define t1 7   // Toggle switch 1
#define t2 4   // Toggle switch 2
#define b1 8   // Button 1
#define b2 9   // Button 2
#define b3 2   // Button 3
#define b4 3   // Button 4
#define LED_PIN 13 // LED pin for blinking

int buttonStates[4];
int pot1 = 0;
int pot2 = 0;

// For button states and debouncing
bool bot1State = false, bot2State = false, bot3State = false, bot4State = false; // Stores the toggled state (1 or 0)
bool bot1Prev = false, bot2Prev = false, bot3Prev = false, bot4Prev = false; // Tracks the previous state of the buttons

// Debounce delay time
unsigned long debounceDelay = 40; 
unsigned long lastDebounceTime[4] = {0, 0, 0, 0}; // Tracks debounce timing for each button

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float angleX, angleY;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY;
float elapsedTime, currentTime, previousTime;
int c = 0;

const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver!!!

RF24 radio(5, 6); // select nRF24L01 CE and CSN  pins

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channels
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

MyData data; // Create a variable with the above structure

void resetData() {
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
  pot1 = 0;
  pot2 = 0;
  // Initialize buttons
  for (int i = 0; i < 4; i++) {
    buttonStates[i] = 0; 
  }
}

void setup() {
  Serial.begin(9600);
  // Initialize interface to the MPU6050
  initialize_MPU6050();
  // Define the radio communication
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  resetData();

  // Activate the Arduino internal pull-up resistors
  pinMode(t1, INPUT_PULLUP);
  pinMode(t2, INPUT_PULLUP);
  pinMode(b1, INPUT_PULLUP);
  pinMode(b2, INPUT_PULLUP);
  pinMode(b3, INPUT_PULLUP);
  pinMode(b4, INPUT_PULLUP);
  //Set LED PIN as output
  pinMode(LED_PIN, OUTPUT);
}

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void handleButtonToggle(int pin, bool &buttonState, bool &previousState, unsigned long &lastTime, int &dataValue) {
  bool currentState = !digitalRead(pin); // Active low buttons, invert the read value

  // Check for state change with debouncing
  if (currentState != previousState) {
    if (millis() - lastTime > debounceDelay) {
      if (currentState == true) { // Button press detected
        buttonState = !buttonState; // Toggle the state
        dataValue = buttonState ? 1 : 0; // Update the data value
      }
      lastTime = millis(); // Reset the debounce timer
    }
  }
  previousState = currentState; // Update the previous state
}

/*// Function to blink the LED a specified number of times
void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100); // LED ON for 100ms
    digitalWrite(LED_PIN, LOW);
    delay(100); // LED OFF for 100ms
  }
}*/

void loop() {
  // Handle button states
  handleButtonToggle(b1, bot1State, bot1Prev, lastDebounceTime[0], buttonStates[0]);
  handleButtonToggle(b2, bot2State, bot2Prev, lastDebounceTime[1], buttonStates[1]);
  handleButtonToggle(b3, bot3State, bot3Prev, lastDebounceTime[2], buttonStates[2]);
  handleButtonToggle(b4, bot4State, bot4Prev, lastDebounceTime[3], buttonStates[3]);

  if(buttonStates[0] == 1){
    calculate_IMU_error();// Use to calculate errors in MPU6050 sensor
    delay(1500);
  }
  else if (buttonStates[1] == 1){
    data.AUX1 = digitalRead(t1);
    data.AUX2 = digitalRead(t2);
    read_IMU(); // Use MPU6050 instead of Joysticks for controling left, right, forward and backward movements
    pot1 = map(analogRead(A7), 0, 1023, 0, 255);
    data.throttle = pot1;
  }
  else if(buttonStates[2] == 1){
    pot2 = map(analogRead(A6), 0, 1023, 0, 255);
    data.throttle = pot2;

    data.AUX1 = digitalRead(t1);
    data.AUX2 = digitalRead(t2);
  }
  else if(buttonStates[3] == 1){
    pot1 = map(analogRead(A7), 0, 1023, 0, 255);
    data.throttle = pot1;
    data.yaw      = mapJoystickValues( analogRead(A2), 12, 523, 1023, true );
    data.pitch    = mapJoystickValues( analogRead(A3), 8, 519, 1023, true );
    data.roll     = mapJoystickValues( analogRead(A1), 19, 523, 1023, true );

    data.AUX1 = digitalRead(t1);
    data.AUX2 = digitalRead(t2);
  }
  else{
    data.throttle = mapJoystickValues( analogRead(A0), 0, 519, 1023, false );
    data.yaw      = mapJoystickValues( analogRead(A2), 12, 523, 1023, true );
    data.pitch    = mapJoystickValues( analogRead(A3), 8, 519, 1023, true );
    data.roll     = mapJoystickValues( analogRead(A1), 19, 523, 1023, true );

    data.AUX1 = digitalRead(t1);
    data.AUX2 = digitalRead(t2);
  }
  radio.write(&data, sizeof(MyData));
}

void initialize_MPU6050() {
  Wire.begin(); // Initialize comunication
  Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); // Talk to the register 6B
  Wire.write(0x00); // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);//end the transmission

  // Accelerometer configuration
  Wire.beginTransmission(MPU);
  Wire.write(0x1C); //Talk to the ACCEL_CONFIG register
  Wire.write(0x10);  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Gyro configuration
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10); // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gury data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 32.8);
    GyroErrorY = GyroErrorY + (GyroY / 32.8);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
}
 
void read_IMU() {
  // Read accelerometer data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);// Read 6 registers total, each axis value is stored in 2 registers

  //For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value
 
  // Calculating angle values using
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.30; // AccErrorX ~(-1.30) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 0.31;  // AccErrorY ~(-0.31)
 
  // Read gyro data
  previousTime = currentTime; // Previous time is stored before the actual time read
  currentTime = millis(); // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8;  // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroX = GyroX - 0.28; //// GyroErrorX ~(0.28)
  GyroY = GyroY + 0.23; // GyroErrorY ~(-0.23)
 
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = GyroX * elapsedTime;
  gyroAngleY = GyroY * elapsedTime;
 
  // Complementary filter - combine accelerometer and gyro angle values
  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;
 
  // Map the angle values from -90deg to +90 deg into values from 0 to 255, like the values we are getting from the Joystick
  data.roll = map(angleX, -90, 90, 255, 0);
  data.pitch = map(angleY, -90, 90, 0, 255);
}

