/*
  Hello everyone this is the code used in the sword .
  if you found this code somewhere other than my video, make sure
  to check it at exampleurl.com

  my mail address: chiseleddasher@gmail.com

 -Netab
*/

/*
  There are 5 functions in this code;
  * updateMPU: updates the gyroscope and accelerometer values from the MPU6050 and filters it with the kalman filter.
  * debug: is used to print the gyro and accel. values in the serial monitor for debugging purposes
  * slash: sends "u" through bluetooth if the correct conditions are met.
  * thrust: sends "o" through bluetooth if the correct conditions are met.
  * buttons: sends assigned letters when corresponding buttons are pressed.
*/

#include <Wire.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <SoftwareSerial.h>
#include <math.h>

// Kalman filter parameters
const float Q = 0.1;
const float R = 5;
const float P = 1000;

// Kalman filters for accelerometer (one per axis)
SimpleKalmanFilter kalmanAccX(Q, R, P);
SimpleKalmanFilter kalmanAccY(Q, R, P);
SimpleKalmanFilter kalmanAccZ(Q, R, P);

// Kalman filters for gyroscope (one per axis)
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

const int buttonA = 6;
const int buttonB = 7;
const int buttonX = 5;
const int buttonY = 8;

// start button (menu)
const int buttonS = 10;

//right joystick (R3) button
const int buttonR = 9;
const int axisY = A2;
const int axisX = A3;

//last state of the button to make them send input only once when pressed
int lastA = HIGH;
int lastB = HIGH;
int lastX = HIGH;
int lastY = HIGH;
int lastR = HIGH;
int lastS = HIGH;

// we record the last states of the joystick too so instead of continuously printing our axis
// we can print whether it was pressed or released
int lastYU = LOW; // Y up
int lastYD = LOW; // Y down
int lastXL = LOW; // X left
int lastXR = LOW; // X right

int cooldown = 0; // attacking cooldown

// Calculate the magnitude of a 3D vector
float calculateMagnitude(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

// Reads and processes the MPU6050 sensor data,
// most of this function was not written by me so tell me at exampleurl.com if there is something wrong
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

void Slash() {
  if (cooldown < 1) {
    if (filteredGyroZ > 130.00) {
      Serial.println("u");

      cooldown = 7;
    }
  }
  else {
    cooldown = cooldown - 1;
  }
}

void Thrust() {
  if (cooldown < 1) {
    if (filteredAccelY < -7.25 && filteredGyroZ < 50) {
      Serial.println("o");

      cooldown = 7;
    }
  }
}

void Buttons() {
  int stateA = digitalRead(buttonA);
  int stateB = digitalRead(buttonB);
  int stateX = digitalRead(buttonX);
  int stateY = digitalRead(buttonY);
  int stateR = digitalRead(buttonR);
  int stateS = digitalRead(buttonS);

  int JoyY = analogRead(axisY);
  int JoyX = analogRead(axisX);

  int JoyYU, JoyYD, JoyXL, JoyXR;

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

  if (JoyY > 600) {
    JoyYD = HIGH;
    JoyYU = LOW;
  } else if (JoyY < 425) {
    JoyYU = HIGH;
    JoyYD = LOW;
  } else {
    JoyYD = LOW;
    JoyYU = LOW;
  }

  if (JoyXL == HIGH && lastXL == LOW) Serial.println("Pl");
  else if (JoyXL == LOW && lastXL == HIGH) Serial.println("Rl");

  if (JoyXR == HIGH && lastXR == LOW) Serial.println("Pj");
  else if (JoyXR == LOW && lastXR == HIGH) Serial.println("Rj");

  if (JoyYU == HIGH && lastYU == LOW) Serial.println("Pi");
  else if (JoyYU == LOW && lastYU == HIGH) Serial.println("Ri");

  if (JoyYD == HIGH && lastYD == LOW) Serial.println("Pk");
  else if (JoyYD == LOW && lastYD == HIGH) Serial.println("Rk");

  // A button
  if (stateA == LOW && lastA == HIGH) Serial.println("Pc");  // pressed A
  else if (stateA == HIGH && lastA == LOW) Serial.println("Rc");  // released A

  // B button (already works as holdable)
  if (stateB == LOW && lastB == HIGH) Serial.println("Px");
  else if (stateB == HIGH && lastB == LOW) Serial.println("Rx");

  // X button
  if (stateX == LOW && lastX == HIGH) Serial.println("Pv");
  else if (stateX == HIGH && lastX == LOW) Serial.println("Rv");

  // Y button
  if (stateY == LOW && lastY == HIGH) Serial.println("Pz");
  else if (stateY == HIGH && lastY == LOW) Serial.println("Rz");

  // Start button
  if (stateS == LOW && lastS == HIGH) Serial.println("Pb");
  else if (stateS == HIGH && lastS == LOW) Serial.println("Rb");

  // Joystick button
  if (stateR == LOW && lastR == HIGH) Serial.println("Pm");
  else if (stateR == HIGH && lastR == LOW) Serial.println("Rm");

  // Update last states
  lastA = stateA;
  lastB = stateB;
  lastX = stateX;
  lastY = stateY;
  lastR = stateR;
  lastS = stateS;
  lastXL = JoyXL;
  lastXR = JoyXR;
  lastYU = JoyYU;
  lastYD = JoyYD;
}

void setup() {
  //normal buttons(A,B,X,Y)
  pinMode(buttonA, INPUT_PULLUP);
  pinMode(buttonB, INPUT_PULLUP);
  pinMode(buttonX, INPUT_PULLUP);
  pinMode(buttonY, INPUT_PULLUP);

  //start button
  pinMode(buttonS, INPUT_PULLUP);

  //joystick button
  pinMode(buttonR, INPUT_PULLUP);

  Serial.begin(9600);
  BTSerial.begin(9600); // default HC05 baud rate is 9600
  Wire.begin();
  mpu.initialize();

  // check if MPU6050 is connected properly
  while (!mpu.testConnection()) {
    delay(1000);
    mpu.initialize();

    Serial.println("trying to initialize mpu");
  }
  Serial.println("Setup finished properly");
}

void loop() {
  updateMPU();

  //Debug(); // uncomment this function if you want to print the mpu6050 values

  Slash();
  Thrust();
  Buttons();

  // forward input coming from the left hand controller (shield) to pc
  while (BTSerial.available()) {
    Serial.write(BTSerial.read());
  }

  delay(10);
}
