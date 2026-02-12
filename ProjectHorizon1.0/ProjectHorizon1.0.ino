#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//
Adafruit_LSM6DSO32 dso32;    // Adafruit LSM6DS Library Use for Accelerometer
SoftwareSerial MyBlue(2, 3); // RX | TX 

// Calibration Variables
float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;
int num_samples = 500;

// PID Variables
float current_error, previous_error = 0;
float prop_error = 0;
float total_integrated_error, integral_error = 0;
float derivative_error = 0;
float motor_control, motor_speed = 0;

// PID Gains
float Kp = 15;
float Ki = 40;
float Kd = 2;

float launch_thresh = 9.81 * 5; // Threshold for when sensor detects launch, g * Number


enum {
  prelaunch,
  launch,
  coast,
  descend
} state = prelaunch;

float PIDController(float error);
void LSM6DSO32Setup();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void) {
  Serial.begin(115200);
  MyBlue.begin(115200); 


  while (!Serial) {
    delay(10);
  }

  // LSM connections and calibration
  Serial.println("LSM6DSO32 test!");
  LSM6DSO32Setup();
  calibrategyro();

  Serial.println("");\

  //Initialize Pin for motor direction control
  pinMode(B11, OUTPUT);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t accel, gyro, temp;
  unsigned long last_time = millis();

  dso32.getEvent(&accel, &gyro, &temp);

  gyro.gyro.x -= gyroX_offset;
  gyro.gyro.y -= gyroY_offset;
  gyro.gyro.z -= gyroZ_offset;


  current_error = gyro.gyro.z;

  while (state == prelaunch) {
    //If 3 seconds have passed
    if(last_time > 3000)
    {
      MyBlue.print("Temperature: ");
      MyBlue.print(temp.temperature);
      MyBlue.print(" °C");
      MyBlue.println();
    }

    if(sqrt(pow(accel.acceleration.x,2) + pow(accel.acceleration.y,2) + pow(accel.acceleration.z,2)) >= launch_thresh){
      state == launch;
    }
  }

  while(state == launch){


  }

  while(state == coast) {
    motor_control = PIDController(current_error, previous_error);

      if (motor_control < 1) {
        digitalWrite(B11, HIGH);
      }       
    else digitalWrite(B11, LOW);

    //mapping motor_control value to PWM duty cycle
    motor_speed = map(abs(motor_control), 0, 32767, 0, 255);
    analogWrite(A0, motor_speed);
  }

  //Save current error for derivative use
  previous_error = current_error;
}

void LSM6DSO32Setup() {
  if (!dso32.begin_I2C()) {
    // if (!dso32.begin_SPI(LSM_CS)) {
    // if (!dso32.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSO32 chip");
    while (1) {
      Serial.println("Failed to find LSM6DSO32 chip");
      delay(10);
    }
  }
  Serial.println("LSM6DSO32 Found!");

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

  Serial.print("Accelerometer range set to: ");
  switch (dso32.getAccelRange()) {
    case LSM6DSO32_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case LSM6DSO32_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case LSM6DSO32_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
    case LSM6DSO32_ACCEL_RANGE_32_G:
      Serial.println("+-32G");
      break;
  }

  // dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (dso32.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      Serial.println("125 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
  }

  // dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (dso32.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
  }

  // dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (dso32.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
  }
}

void calibrategyro() {
  Serial.println("Keep device still. Calibrating Gyro...");
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;

  for (int i = 0; i < num_samples; i++) {
    sensors_event_t accel, gyro, temp;
    dso32.getEvent(&accel, &gyro, &temp);

    x_sum += gyro.gyro.x;
    y_sum += gyro.gyro.y;
    z_sum += gyro.gyro.z;
    delay(2);  // Small delay to prevent reading same register twice instantly
  }

  gyroX_offset = x_sum / num_samples;
  gyroY_offset = y_sum / num_samples;
  gyroZ_offset = z_sum / num_samples;

  Serial.print("Calibration Complete. Z Offset: ");
  Serial.println(gyroZ_offset);
}

float PIDController(float error, float prev_error) {
  // Proportional gain
  prop_error = Kp * error;
  // Integral Error
  total_integrated_error += error;
  integral_error = Ki * error;
  // Derivative Error
  derivative_error = Kd * (error - prev_error);
}
