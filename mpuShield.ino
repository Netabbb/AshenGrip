/*
  Hello everyone this is the code used in the shield .
  if you found this code somewhere other than my video, make sure
  to check it at exampleurl.com

  my mail address: chiseleddasher@gmail.com

 -Netab
*/

/*
  There are 5 functions in this code;
  * updateMPU: updates the gyroscope and accelerometer values from the MPU6050 and filters it with the kalman filter.
  * debug: is used to print the gyro and accel. values in the serial monitor for debugging purposes
  * block: sends "Pe" and "Re" through bluetooth if the correct conditions are met. (P stands for press and R stands for release)
  * parry: sends "q" through bluetooth if the correct conditions are met.
  * buttons: sends assigned letters when corresponding buttons are pressed.
*/

#include <Wire.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <SoftwareSerial.h>  // Include SoftwareSerial library for communication

// Kalman filter parameters
const float Q = 0.1;
const float R = 5;
const float P = 1000;

// Kalman filter for accelerometer (one per axis)
SimpleKalmanFilter kalmanAccX(Q, R, P);
SimpleKalmanFilter kalmanAccY(Q, R, P);
SimpleKalmanFilter kalmanAccZ(Q, R, P);

// Kalman filter for gyroscope (one per axis)
SimpleKalmanFilter kalmanGyroX(Q, R, P);
SimpleKalmanFilter kalmanGyroY(Q, R, P);
SimpleKalmanFilter kalmanGyroZ(Q, R, P);

MPU6050 mpu;
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

float accelX_f, accelY_f, accelZ_f;
float gyroX_f, gyroY_f, gyroZ_f;

float filteredAccelX, filteredAccelY, filteredAccelZ;
float filteredGyroX, filteredGyroY, filteredGyroZ;

const float gravity = 9.81;
unsigned long lastTime = 0;
unsigned long currentTime = 0;
float deltaTime = 0;

// MPU6050 sensitivity
const float accelSensitivity = 0.00058;
const float gyroSensitivity = 250.0;

// For Bluetooth communication, use SoftwareSerial for HC05
SoftwareSerial BTSerial(2, 3);  // Pin 2 -> RX, Pin 3 -> TX (connect HC05 TX to Arduino pin 2, RX to pin 3)

const int threshold = 1;
const float stationaryMin = -10.00;
const float stationaryMax = 10.00;
const float movementThreshold = 1.50;
float magnitude;

//dpad buttons
const int buttonDown = 12;
const int buttonRight = 13;
const int buttonLeft = 11;
const int buttonUp = 10;

//left joystick (L3) button
const int buttonL = 9;
const int axisY = A1;
const int axisX = A2;

//last state of the button to make them send input only once when pressed
int lastDown = HIGH;
int lastRight = HIGH;
int lastLeft = HIGH;
int lastUp = HIGH;
int lastL = HIGH;

// we record the last states of the joystick too so instead of continuously printing our axis and flooding the serial communication
// we can print whether it was pressed or released
int lastYU = LOW; // Y up
int lastYD = LOW; // Y down
int lastXL = LOW; // X left
int lastXR = LOW; // X right

//counter for how many cycles the shield has been held in blocking position
int counter = 0;

int lastShield = LOW; // using this for the same reason as the joysticks

//cooldown for parry function x20ms
int cooldown = 0;

float calculateMagnitude(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

// Reads and processes the MPU6050 sensor data,
// most of this function was not written by me so tell me at exampleurl.com if there is something wrong
// also the gyro readings may not be needed but I kept them just in case
void updateMPU() {
  currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;  // in seconds

  // Read raw sensor data
  mpu.getAcceleration(&accelX, &accelY, &accelZ);
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);

  // Convert raw accelerometer data
  accelX_f = accelX * accelSensitivity;
  accelY_f = accelY * accelSensitivity;
  accelZ_f = accelZ * accelSensitivity;

  // Compute magnitude before gravity compensation
  magnitude = calculateMagnitude(accelX_f, accelY_f, accelZ_f);

  // Gravity Compensation: remove a proportion of gravity
  float gravityMagnitude = sqrt(accelX_f * accelX_f + accelY_f * accelY_f + accelZ_f * accelZ_f);
  accelX_f -= accelX_f * (gravityMagnitude - gravity) / gravityMagnitude;
  accelY_f -= accelY_f * (gravityMagnitude - gravity) / gravityMagnitude;
  accelZ_f -= accelZ_f * (gravityMagnitude - gravity) / gravityMagnitude;

  // Apply Kalman filters to accelerometer data
  filteredAccelX = kalmanAccX.updateEstimate(accelX_f);
  filteredAccelY = kalmanAccY.updateEstimate(accelY_f);
  filteredAccelZ = kalmanAccZ.updateEstimate(accelZ_f);

  // Convert raw gyroscope data to degrees per second
  gyroX_f = gyroX / gyroSensitivity;
  gyroY_f = gyroY / gyroSensitivity;
  gyroZ_f = gyroZ / gyroSensitivity;

  // Apply Kalman filters to gyroscope data
  filteredGyroX = kalmanGyroX.updateEstimate(gyroX_f);
  filteredGyroY = kalmanGyroY.updateEstimate(gyroY_f);
  filteredGyroZ = kalmanGyroZ.updateEstimate(gyroZ_f);

  // In my case these offsets are needed to make the reading more accurate
  filteredGyroX += 1;
  filteredGyroY += 1;

  // the readings are noisy so we ignore small changes by setting it to zero
  if (abs(filteredGyroX) <= threshold) filteredGyroX = 0;
  if (abs(filteredGyroY) <= threshold) filteredGyroY = 0;
  if (abs(filteredGyroZ) <= threshold) filteredGyroZ = 0;

  lastTime = currentTime;
}

//use this function if you want to see the mpu6050 values in the serial monitor
void Debug() {
  // gyro values in degrees/s
  Serial.print("gyroX: ");
  Serial.print(filteredGyroX);
  Serial.print("gyroY: ");
  Serial.print(filteredGyroY);
  Serial.print("gyroZ: ");
  Serial.println(filteredGyroZ);

  // accelerometer values in m/s^2
  Serial.print("AccelX: ");
  Serial.print(filteredAccelX);
  Serial.print("AccelY: ");
  Serial.print(filteredAccelY);
  Serial.print("AccelZ: ");
  Serial.println(filteredAccelZ);
}

// if the shield is held in blocking positions for x cycles(counter) it sends blocking input
void Block() {
  int shield;

  if (counter >= 3) {
    shield = HIGH;
  }
  else {
    shield = LOW;
  }

  if (shield == HIGH && lastShield == LOW) {
    BTSerial.println("Pe");
  }
  else if (shield == LOW && lastShield == HIGH ) {
    BTSerial.println("Re");
  }

  if (filteredAccelY < 10.10 && filteredAccelY > 9.50 &&
      filteredAccelX < 2.00 && filteredAccelX > -2.00 &&
      filteredAccelZ < 2.00 && filteredAccelZ > -2.00)
  {
    counter++;
  }
  else {
    counter = 0;
  }

  lastShield = shield;
}

//there is a cooldown because sometimes it sends q more than once
void Parry() {
  if (cooldown < 1) {
    if (filteredAccelX < -6.70) {
      BTSerial.println("q");

      cooldown = 3; // x10ms (loop delay)
    }
  }
  else {
    cooldown = cooldown - 1;
  }
}

void Buttons() {
  int stateDown = digitalRead(buttonDown);
  int stateRight = digitalRead(buttonRight);
  int stateLeft = digitalRead(buttonLeft);
  int stateUp = digitalRead(buttonUp);
  int stateL = digitalRead(buttonL);

  int JoyY = analogRead(axisY);
  int JoyX = analogRead(axisX);

  int JoyYU;
  int JoyYD;
  int JoyXL;
  int JoyXR;

  // Process Joystick X
  if (JoyX > 600) {
    JoyXL = HIGH;
    JoyXR = LOW;
  } else if (JoyX < 425) {
    JoyXR = HIGH;
    JoyXL = LOW;
  } else {
    JoyXL = LOW;
    JoyXR = LOW;
  }

  // Process Joystick Y
  if (JoyY > 600 ) {
    JoyYD = HIGH;
    JoyYU = LOW;
  } else if (JoyY < 425) {
    JoyYU = HIGH;
    JoyYD = LOW;
  } else {
    JoyYD = LOW;
    JoyYU = LOW;
  }

  // Joystick direction presses and releases
  if (JoyXL == HIGH && lastXL == LOW) BTSerial.println("Pd");
  else if (JoyXL == LOW && lastXL == HIGH) BTSerial.println("Rd");

  if (JoyXR == HIGH && lastXR == LOW) BTSerial.println("Pa");
  else if (JoyXR == LOW && lastXR == HIGH) BTSerial.println("Ra");

  if (JoyYU == HIGH && lastYU == LOW) BTSerial.println("Ps");
  else if (JoyYU == LOW && lastYU == HIGH) BTSerial.println("Rs");

  if (JoyYD == HIGH && lastYD == LOW) BTSerial.println("Pw");
  else if (JoyYD == LOW && lastYD == HIGH) BTSerial.println("Rw");

  // D-Pad button press and release
  if (stateDown == LOW && lastDown == HIGH) BTSerial.println("Pg");
  else if (stateDown == HIGH && lastDown == LOW) BTSerial.println("Rg");

  if (stateRight == LOW && lastRight == HIGH) BTSerial.println("Ph");
  else if (stateRight == HIGH && lastRight == LOW) BTSerial.println("Rh");

  if (stateLeft == LOW && lastLeft == HIGH) BTSerial.println("Pf");
  else if (stateLeft == HIGH && lastLeft == LOW) BTSerial.println("Rf");

  if (stateUp == LOW && lastUp == HIGH) BTSerial.println("Pt");
  else if (stateUp == HIGH && lastUp == LOW) BTSerial.println("Rt");

  // Joystick button (L3)
  if (stateL == LOW && lastL == HIGH) BTSerial.println("Pr");
  else if (stateL == HIGH && lastL == LOW) BTSerial.println("Rr");

  // Update last states
  lastDown = stateDown;
  lastRight = stateRight;
  lastLeft = stateLeft;
  lastUp = stateUp;
  lastL = stateL;
  lastXL = JoyXL;
  lastXR = JoyXR;
  lastYU = JoyYU;
  lastYD = JoyYD;
}

void setup() {
  //dpad buttons
  pinMode(buttonDown, INPUT_PULLUP);
  pinMode(buttonRight, INPUT_PULLUP);
  pinMode(buttonLeft, INPUT_PULLUP);
  pinMode(buttonUp, INPUT_PULLUP);

  //joystick button
  pinMode(buttonL, INPUT_PULLUP);

  Serial.begin(9600);
  BTSerial.begin(9600);  // HC05 default baud rate is 9600
  Wire.begin();
  mpu.initialize();

  // check if MPU6050 is connected properly
  while (!mpu.testConnection()) {
    delay(1000);
    mpu.initialize();
    BTSerial.println("trying to initialize mpu");
  }
  BTSerial.println("setup finished properly");
}

void loop() {
  updateMPU();

  //Debug(); // uncomment this function if you want to print the mpu6050 values

  Parry();
  Block();
  Buttons();

  delay(10);
}
