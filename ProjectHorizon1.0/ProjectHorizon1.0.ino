#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//
Adafruit_LSM6DSO32 dso32;    // Adafruit LSM6DS Library Use for Accelerometer
SoftwareSerial MyBlue(2, 3); // RX | TX 

enum {
  prelaunch,
  launch,
  coast,
  descend
} state = prelaunch;

typedef struct {
  float Q_angle;   // Process noise: Uncertainty in the angle evolution
  float Q_bias;    // Process noise: Uncertainty in the gyro bias
  float R_measure; // Measurement noise: Uncertainty in the accelerometer measure

  float angle;     // The calculated angle (OUTPUT)
  float bias;      // The calculated gyro bias
  float rate;      // Unbiased rate

  float P[2][2];   // Error covariance matrix
} Kalman_t;

// Kalman Filter
Kalman_t KalmanX; // For Roll
Kalman_t KalmanY; // For Pitch
uint32_t timer;

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

float PIDController(float error); //calculates the motor controll signal
void LSM6DSO32Setup(); //Sets up the IMU, chooses properties of IMU
void calibrategyro(); //Calibrates IMU gyro to deal with gyro drift
void Kalman_Init(Kalman_t *kf); //Kalman filter initialization to determine roll and pitch angles
float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt); //Kalman filter  to determine roll and pitch angles

///////////////// Main //////////////////////////////////////////////////////////////////////////////////////////////
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

  // Initialize Kalman filter parameters
  Kalman_Init(&KalmanX);
  Kalman_Init(&KalmanY);
  timer = micros(); // Start timer

  Serial.println("");

  //Initialize Pin for motor direction control
  pinMode(B11, OUTPUT);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  unsigned long last_time = millis();
  double dt = (double)(micros() - timer) / 1000000.0;
  timer = micros();

  dso32.getEvent(&accel, &gyro, &temp);

  float gyroRateX = (gyro.gyro.x - gyroX_offset) * 57.29578f;
  float gyroRateY = (gyro.gyro.y - gyroY_offset) * 57.29578f;

  float accel_mag =sqrt(pow(accel.acceleration.x,2) + pow(accel.acceleration.y,2) + pow(accel.acceleration.z,2));
  float accRoll  = atan2(accel.acceleration.y, accel.acceleration.z) * 57.29578f;
  float accPitch = atan2(-accel.acceleration.x, sqrt(pow(accel.acceleration.y,2) + pow(accel.acceleration.z, 2))) * 57.29578f;

  float roll  = Kalman_GetAngle(&KalmanX, accRoll, gyroRateX, dt);
  float pitch = Kalman_GetAngle(&KalmanY, accPitch, gyroRateY, dt);

  current_error = (gyro.gyro.z - gyroZ_offset) * 57.29578f;

  switch(state){
    case prelaunch:
      if(millis() - last_time > 1000)
      {
        // 2. Print Acceleration Data
        MyBlue.print("Acc[X,Y,Z]: [");
        MyBlue.print(accel.acceleration.x); MyBlue.print(", ");
        MyBlue.print(accel.acceleration.y); MyBlue.print(", ");
        MyBlue.print(accel.acceleration.z); MyBlue.print("]\t");
        MyBlue.print("Mag: "); MyBlue.print(accel_mag); MyBlue.print("\t");

        // 3. Print Orientation & Temp
        MyBlue.print("Temp: "); MyBlue.print(temp.temperature); MyBlue.print("°C\t");
        MyBlue.print("Roll: "); MyBlue.print(roll); MyBlue.print("°\t");
        MyBlue.print("Pitch: "); MyBlue.print(pitch); MyBlue.print("°\t");
      }

      // If in prelaunch, gravity magnitude should be ~9.81. If it's off by more than 2 m/s^2, flag it.
      if (state == prelaunch && abs(accel_mag - 9.81) > 2.0) {
       MyBlue.print("[FLAG: BAD_PAD_CALIBRATION] ");
      }

      if(accel_mag >= launch_thresh){
        state = launch;
      }
    break;
    case launch:
    break;
    case coast:
    break;
    case descend:
    break;
  }

  if(state == launch){


  }

  if(state == coast) {
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

////////////////////// Functions ////////////////////////////////////////////////////////////
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

  float control_output = prop_error + integral_error + derivative_error;
  return control_output;
}

void Kalman_Init(Kalman_t *kf) {
    // Default tuning parameters
    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.03f;

    kf->angle = 0.0f;
    kf->bias = 0.0f;

    kf->P[0][0] = 0.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 0.0f;
}

float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt) {
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    // Update estimation error covariance
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // Calculate Kalman Gain
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Calculate Angle Difference (Measurement - Predicted)
    float y = newAngle - kf->angle;

    // Correct the state
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    // Update error covariance
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}