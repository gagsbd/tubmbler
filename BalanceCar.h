/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-08 09:35:07
 * @LastEditTime: 2019-10-11 16:25:04
 * @LastEditors: Please set LastEditors
 */
#include "MsTimer2.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
#include "EnableInterrupt.h"
//#include "MPU6050_6Axis_MotionApps20.h"

#include "MPU6050.h"
#include "Wire.h"
MPU6050 mpu;
KalmanFilter kalmanfilter;

//Setting PID parameters

double kp_balance = 55, kd_balance = 0.75;
double kp_speed = 10, ki_speed = 0.26;
double kp_turn = 2.5, kd_turn = 0.5;

//Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration

volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
volatile  float encoder_target = 0;
int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;
double speed_control_output = 0;
double rotation_control_output = 0;
double speed_filter = 0;
int speed_control_period_count = 0;
double car_speed_integeral = 0;
double speed_filter_old = 0;
int setting_car_speed = 0;
int setting_turn_speed = 0;
double pwm_left = 0;
double pwm_right = 0;
float kalmanfilter_angle;
// char balance_angle_min = -27;
// char balance_angle_max = 27;
char balance_angle_min = -22;
char balance_angle_max = 22;


// **** new vars
bool hasStopped = true;
long timerUpdate = 0;
bool isTrapezoidalMotion = false;  // Flag to prevent updateStatus() interference

int enc_per_rotation = 22;
int accelRate = 200;
int decelRate = 200;

void carStop()
{
  Serial.write("stop");
  digitalWrite(AIN1, LOW); //HIGH
  digitalWrite(BIN1, LOW);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
  hasStopped = true;
}

void carHardStop()
{
   Serial.write("hardstop");
  // Stronger stop for end-of-motion braking
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 255);
  analogWrite(PWMB_RIGHT, 255);
  delay(10);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
  hasStopped = true;
}

void carSoftStop()
{
  // Gradually increase brake duty to reduce jerk
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  for (int duty = 80; duty <= 255; duty += 40) {
    analogWrite(PWMA_LEFT, duty);
    analogWrite(PWMB_RIGHT, duty);
    delay(15);
  }
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
  hasStopped = true;
}

void carForward(unsigned char speed)
{
  if(!hasStopped)
  {
    return;
  }
  Serial.println("Forward - " + String(speed));
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
  hasStopped = false;
}

void carForwardTrapezoidal(long leftDist, long rightDist, int maxSpeed)
{
    if (!hasStopped)
        return;

    // -----------------------------
    // 1. Normalize inputs
    // -----------------------------
    long L_target = abs(leftDist);
    long R_target = abs(rightDist);

    int L_dir = (leftDist  >= 0) ? LOW : HIGH;
    int R_dir = (rightDist >= 0) ? LOW : HIGH;

    digitalWrite(AIN1, L_dir);
    digitalWrite(BIN1, R_dir);

    // -----------------------------
    // 2. Compute trapezoidal profile for both motors
    // Reduce decel rate to extend coast-down and maintain balance
    // -----------------------------
    float tAccel  = (float)maxSpeed / accelRate;
    float tDecel  = (float)maxSpeed / (decelRate / 2.5f);  // Slower decel for smooth stop

    float distAccel = 0.5f * maxSpeed * tAccel;
    float distDecel = 0.5f * maxSpeed * tDecel;

    float L_cruise = L_target - distAccel - distDecel;
    float R_cruise = R_target - distAccel - distDecel;

    // If either motor cannot reach full speed → triangle profile
    if (L_cruise < 0 || R_cruise < 0) {
        distAccel = distDecel = (float)min(L_target, R_target) * 0.5f;
        tAccel = sqrt(2.0f * distAccel / accelRate);
        tDecel = sqrt(2.0f * distDecel / (decelRate / 2.5f));
        L_cruise = R_cruise = 0;
    }

    float tCruise = (float)min(L_cruise, R_cruise) / maxSpeed;

    unsigned long tAccelEnd  = tAccel  * 1e6;
    unsigned long tCruiseEnd = tAccelEnd + tCruise * 1e6;
    unsigned long tDecelEnd  = tCruiseEnd + tDecel * 1e6;
    unsigned long tCoastEnd  = tDecelEnd + 500000;  // 500ms coast-down to ~0 at end

    // -----------------------------
    // 3. Reset encoder counters & yaw tracking
    // -----------------------------
    encoder_count_left_a  = 0;
    encoder_count_right_a = 0;
    
    // Store initial yaw for straight-line correction
    extern float yaw;
    float yaw_target = yaw;  // Target heading - stay on this line
    float yaw_integral = 0;

    unsigned long startT = micros();
    hasStopped = false;
    isTrapezoidalMotion = true;  // Prevent updateStatus() from interfering

    // Yaw correction PID gains (tuned for gyro feedback)
    float kp_yaw = 0.5f;     // Proportional gain for yaw error
    float ki_yaw = 0.1f;     // Integral gain for sustained drift
    float kd_yaw = 0.2f;     // Derivative gain to damp oscillation

    // -----------------------------
    // 4. Main motion loop
    // Exits when target reached + coast phase complete
    // -----------------------------
    while (true)
    {
        unsigned long t = micros() - startT;
        float speedCmd;

        // Phase selection
        if (t < tAccelEnd) {
            speedCmd = accelRate * (t / 1e6f);
        }
        else if (t < tCruiseEnd) {
            speedCmd = maxSpeed;
        }
        else if (t < tDecelEnd) {
            float td = (t - tCruiseEnd) / 1e6f;
            speedCmd = maxSpeed - (decelRate / 2.5f) * td;
        }
        else if (t < tCoastEnd) {
            // Coast phase: very gradual ramp to near-zero
            // Maintains balance by keeping motors running at minimal speed
            float tCoast = (t - tDecelEnd) / 1e6f;
            speedCmd = maxSpeed * (1.0f - (tCoast * tCoast) / 0.25f);  // Smooth quadratic decay
            speedCmd = constrain(speedCmd, 5, maxSpeed);  // Minimum 5 to maintain balance control
        }
        else {
            speedCmd = 0;
        }

        // Get fresh gyro data for yaw correction
        extern float yaw;
        extern KalmanFilter kalmanfilter;
        
        // Yaw error (how much robot has drifted from target heading)
        float yaw_error = yaw - yaw_target;
        
        // Constrain error to ±180 degrees
        if (yaw_error > 180.0f) yaw_error -= 360.0f;
        if (yaw_error < -180.0f) yaw_error += 360.0f;
        
        // PID accumulation
        yaw_integral += yaw_error * (t / 1e6f);
        yaw_integral = constrain(yaw_integral, -10.0f, 10.0f);
        
        float yaw_derivative = kalmanfilter.Gyro_z;  // Rate of change from gyro
        
        // Calculate yaw correction (negative = turn left, positive = turn right)
        float yaw_correction = kp_yaw * yaw_error + ki_yaw * yaw_integral - kd_yaw * yaw_derivative;
        yaw_correction = constrain(yaw_correction, -20.0f, 20.0f);

        // -----------------------------
        // 5. Encoder-based synchronization + Yaw correction
        // Stop if target reached AND coast phase is complete
        // -----------------------------
        long L_err = L_target - encoder_count_left_a;
        long R_err = R_target - encoder_count_right_a;

        if (L_err <= 0 && R_err <= 0 && t >= tCoastEnd)
            break;

        // Encoder sync correction
        float syncGain = 0.002f;
        float diff = (float)(encoder_count_left_a - encoder_count_right_a);
        float L_speed = speedCmd + diff * syncGain;
        float R_speed = speedCmd - diff * syncGain;

        // Apply yaw correction: slow right motor to turn left (positive error), vice versa
        L_speed += yaw_correction;
        R_speed -= yaw_correction;

        // clamp
        L_speed = constrain(L_speed, 0, maxSpeed);
        R_speed = constrain(R_speed, 0, maxSpeed);

        // convert to PWM
        int L_pwm = map(L_speed, 0, maxSpeed, 0, 255);
        int R_pwm = map(R_speed, 0, maxSpeed, 0, 255);

        analogWrite(PWMA_LEFT,  L_pwm);
        analogWrite(PWMB_RIGHT, R_pwm);
    }

    // -----------------------------
    // 6. Stop motors - now transition to balanceCar() control
    // Stop by coasting to zero output instead of using an abrupt brake pulse
    // -----------------------------
    analogWrite(PWMA_LEFT, 0);
    analogWrite(PWMB_RIGHT, 0);
    
    // Debug: Show actual encoder counts
    extern float yaw;
    Serial.print(F("Motion complete - Left encoder: "));
    Serial.print(encoder_count_left_a);
    Serial.print(F(" | Right encoder: "));
    Serial.print(encoder_count_right_a);
    Serial.print(F(" | Yaw offset: "));
    Serial.println(yaw);
    
    // Reset motion control variables so balance loop takes over
    setting_car_speed = 0;
    setting_turn_speed = 0;
    car_speed_integeral = 0;
    isTrapezoidalMotion = false;  // Allow updateStatus() to work again

    hasStopped = true;
}

void carBack(unsigned char speed)
{
  if(!hasStopped)
  {
    return;
  }

  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
  hasStopped = false;
}
void turnRight()
{
  if(!hasStopped)
  {
    return;
  }
  Serial.println("Turning right");
  digitalWrite(AIN1, 1);  //rotate reverse
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, 200);
  analogWrite(PWMB_RIGHT, 200);
  hasStopped = false;
}

void turnLeft()
{
  if(!hasStopped)
  {
    return;
  }

  digitalWrite(AIN1, 0);  
  digitalWrite(BIN1, 1);//rotate reverse
  analogWrite(PWMA_LEFT, 200);
  analogWrite(PWMB_RIGHT, 200);
  hasStopped = false;

}

void balanceCar()
{
  sei();
  extern bool isTrapezoidalMotion;
  if (!isTrapezoidalMotion) {
    encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
    encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
    encoder_count_left_a = 0;
    encoder_count_right_a = 0;
  }
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  
  // Integrate gyro Z-axis to track heading (yaw)
  extern float yaw;
  yaw += kalmanfilter.Gyro_z * dt;  // dt = 0.005 seconds (5ms)
  
  double balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);

  speed_control_period_count++; 
  if (speed_control_period_count >= 8)
  {
    speed_control_period_count = 0;
    if (!isTrapezoidalMotion) {
      double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
      encoder_left_pulse_num_speed = 0;
      encoder_right_pulse_num_speed = 0;
      speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
      speed_filter_old = speed_filter;
      car_speed_integeral += speed_filter;
      car_speed_integeral += -setting_car_speed;
      car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
      speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
    } else {
      // During trapezoidal motion, use simplified speed control
      speed_control_output = -setting_car_speed;
    }
    rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
  }

  pwm_left = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))
  {
    motion_mode = STOP;
    Serial.println("Balance 1");
    carStop();
  }

  if (motion_mode == STOP && key_flag != '4')
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
     Serial.println("Balance 2");
    carStop();
  }
  else if (motion_mode == STOP)
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
  }
  else
  {
    if (pwm_left < 0)
    {
      digitalWrite(AIN1, 1);
      analogWrite(PWMA_LEFT, -pwm_left);
    }
    else
    {
      digitalWrite(AIN1, 0);
      analogWrite(PWMA_LEFT, pwm_left);
    }
    if (pwm_right < 0)
    {
      digitalWrite(BIN1, 1);
      analogWrite(PWMB_RIGHT, -pwm_right);
    }
    else
    {
      digitalWrite(BIN1, 0);
      analogWrite(PWMB_RIGHT, pwm_right);
    }
  }
}

void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}

void updateStatus(long usecElapsed)
{
  
  timerUpdate += usecElapsed;
  volatile unsigned int enc_current = (abs(encoder_count_right_a) + abs(encoder_count_left_a))/2;
  if( enc_current >= encoder_target) 
  {
    if(!hasStopped)
    {
    Serial.print(String(enc_current) + "-");
    Serial.println(String(encoder_target) + " - stopping..");
    }
     Serial.println("Update status");
    carStop();
    encoder_count_right_a = 0;
    encoder_count_left_a =0;
    delay(500);
  }
  
}

void carInitialize()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
   Serial.println("carInitialize");
  carStop();
  Wire.begin();
  mpu.initialize();
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderCountLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCountRightA, CHANGE);
 // MsTimer2::set(5, balanceCar);
 // MsTimer2::start();
}
