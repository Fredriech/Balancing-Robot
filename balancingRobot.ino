#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
// Sensor variables
// long acc_x, acc_y, acc_z;
// int gyro_x, gyro_y, gyro_z, temperature;
// long gyro_cal_x, gyro_cal_y, gyro_cal_z;
// long acc_calx, acc_caly, acc_calz;
// float pitch_angle, roll_angle, yaw_angle;
// float acc_pitch_angle, acc_roll_angle;
long looping_time;
// long acc_res_vector;
// float acc_Xg, acc_Yg, acc_Zg;

float output_pitch_angle = 0, output_roll_angle = 0;
// float Vbat;

// Robot system variables
int channel_A[3] = {10, 12, 11};
int channel_B[3] = {4, 5, 6};
// #define rad_to_deg 57.295779

// PID variables
float error, error_sum = 0;
float current_pitch_angle, previous_pitch_angle = 0, change_in_angle;
float pid_output;

#define Kp 14
#define Ki 1.1
#define Kd 8
#define sample_time 0.005
const float target_angle = 1.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // TWBR = 12;
  delay(250);
  mpu.initialize();

  // Initialize motor pins
  for (int i = 0; i < 3; i++) {
    pinMode(channel_A[i], OUTPUT);
    pinMode(channel_B[i], OUTPUT);
  }

  looping_time = micros();
}

void loop() {
  // Read sensor data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert accelerometer values to angle
    float accel_pitch = atan2(ay, az) * 180 / PI;
    
    // Convert gyro readings to deg/s
    float gyro_pitch = gx / 131.0;  

    // Apply complementary filter
    output_pitch_angle = 0.98 * (output_pitch_angle + gyro_pitch * sample_time) + 0.02 * accel_pitch;

  Serial.print("Pitch Angle: ");
  Serial.println(output_pitch_angle);

  // PID control
  PID_CONTROL();

  pid_output = abs(pid_output);

  // Motor control
  if (output_pitch_angle > target_angle + 1) { // Dead zone of ±1 degree
    digitalWrite(channel_A[0], LOW);
    digitalWrite(channel_A[1], HIGH);
    digitalWrite(channel_B[1], LOW);
    digitalWrite(channel_B[0], HIGH);
    analogWrite(channel_A[2], pid_output);
    analogWrite(channel_B[2], pid_output);
    Serial.print(" PID: ");
    Serial.println(pid_output);
  } else if (output_pitch_angle < target_angle - 1) {
    digitalWrite(channel_A[0], HIGH);
    digitalWrite(channel_A[1], LOW);
    digitalWrite(channel_B[1], HIGH);
    digitalWrite(channel_B[0], LOW);
    analogWrite(channel_A[2], pid_output);
    analogWrite(channel_B[2], pid_output);
    Serial.print(" PID: ");
    Serial.println(pid_output);
  } else {
    analogWrite(channel_A[2], 0);
    analogWrite(channel_B[2], 0);
  }

  // Maintain sampling rate
  while (micros() - looping_time < 4000); // Adjust for 250Hz
  looping_time = micros();
}

void PID_CONTROL() {
  current_pitch_angle = output_pitch_angle;
  error = current_pitch_angle - target_angle;
  error_sum += error;
  change_in_angle = current_pitch_angle - previous_pitch_angle; 

  pid_output = (Kp * error) + (Ki * error_sum * sample_time) + ((Kd * change_in_angle) / sample_time);

  previous_pitch_angle = current_pitch_angle; 

  // Clamp PID output
  if (pid_output > 255) pid_output = 255;
  else if (pid_output < -255) pid_output = -255;
}

// void acc_gyro_reading() {
//   Wire.beginTransmission(0x68);
//   Wire.write(0x3B);
//   Wire.endTransmission();
//   Wire.requestFrom(0x68, 14);

//   acc_x = Wire.read() << 8 | Wire.read();
//   acc_y = Wire.read() << 8 | Wire.read();
//   acc_z = Wire.read() << 8 | Wire.read();
//   temperature = Wire.read() << 8 | Wire.read();
//   gyro_x = (Wire.read() << 8 | Wire.read()) / 131;
//   gyro_y = (Wire.read() << 8 | Wire.read()) / 131;
//   gyro_z = (Wire.read() << 8 | Wire.read()) / 131;
// }

// void setup_acc_gyro() {
//   Wire.beginTransmission(0x68);
//   Wire.write(0x6B);
//   Wire.write(0x00);
//   Wire.endTransmission();

//   Wire.beginTransmission(0x68);
//   Wire.write(0x1C);
//   Wire.write(0x01);
//   Wire.endTransmission();

//   Wire.beginTransmission(0x68);
//   Wire.write(0x1B);
//   Wire.write(0x00);
//   Wire.endTransmission();
// }