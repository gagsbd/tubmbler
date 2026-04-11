/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-09-12 14:51:36
 * @LastEditTime: 2019-10-11 16:39:57
 * @LastEditors: Please set LastEditors
 */
#include <Arduino.h>
#include "Pins.h"
#include "mode.h"
#include "Command.h"
#include "BalanceCar.h"
#include "Rgb.h"
#include "Ultrasonic.h"
#include "voltage.h"
#include "EnableInterrupt.h"

//======================================================================================
// Command object is used to hold a list of commands to be executed
//======================================================================================

float yaw;
float targetYaw;
float x=0;
float y=0;
long reverse_enc = 0;
long reverse_last_enc = 0;
int in_reverse = 0;

int left_turn_correction = 4;
int right_turn_correction = 4;
// float x_target;
// float y_target;
// int current_direction = 1;
//int block_width = 500;

#define MAX_COMMANDS    60
#define ENCODER_COUNTS_90_DEG   266
#define ENC_PER_MM  0.25   // Calibrated: ~13665 counts for 500mm → ~27 counts/mm
unsigned long usLast;
unsigned long startTime =0;
long usecElapsed;
long usScanLong;
int  usLongResetCount;
long usScanAvg;
long timerusScan;
int scanCount;

long timerRunTime;
int speedFwd = 200;
int speedTurn;
int flagTimeRun;
int flagLastMoveFwd;

int flagLED;

float sonicDistance;

int printStep;
int printLastCmd;
unsigned long msTimerPrint;

unsigned long msTimerMPU;
unsigned long timerSonicRange;

unsigned long timerPBStartOn;
unsigned long timerPBStartOff;

unsigned long timerDelay;




class carCommand
{
  private:
    int start;
    int end;
    int last_cmd;
    int list[MAX_COMMANDS];
    float p1[MAX_COMMANDS];
    int flagFirstScan;

  public:
    inline int   current()  { return list[start]; };
    inline float getParameter1() { return p1[start]; };
    inline int   empty()    { if (start==end) return 127; else return 0; };
    inline int   last()     { return last_cmd; };
    inline int   firstScan() { int flag = flagFirstScan; flagFirstScan = 0; return flag; } 
  
    carCommand() {
      clear();
    }
  
    void clear() {
      start = 0;
      end = 0;
      last_cmd = 0;
      flagFirstScan = 0;
      int n = 0;
      for (n=0;n<MAX_COMMANDS;n++) { list[n] = 0;  p1[n] = 0; }
    }

    void add(int cmd) {
      add(cmd,0);
    }


    void add(int cmd,float par1) {
      list[end] = cmd;
      p1[end] = par1;
      end++;
      if (end >= MAX_COMMANDS) end = 0;
    }
  
    int next() {
      flagFirstScan = 127;
      if (empty()) return 0;
      last_cmd = list[start];   // save last command
      start++;
      if (start >= MAX_COMMANDS) start = 0;
      return list[start];
    }

};

unsigned long start_prev_time = 0;
boolean carInitialize_en = true;
carCommand cmdQueue;

//
//  List of possible vehicle motion commands
//   -- Additional motion commands can be added which will require code to execute
//
#define VEHICLE_START_WAIT      1       // Wait for the start button to be pressed
#define VEHICLE_START           2       // First motion command after button press
#define VEHICLE_FORWARD         10
#define VEHICLE_REVERSE         20
#define VEHICLE_TURN_RIGHT      40
#define VEHICLE_TURN_LEFT       50
#define VEHICLE_TURN_180        60
#define VEHICLE_SET_MOVE_SPEED  101
#define VEHICLE_SET_TURN_SPEED  102
#define VEHICLE_SET_ACCEL       105
#define VEHICLE_FINISHED        900     // Must be at the end of the command list
#define VEHICLE_STRAIGHT        950
#define VEHICLE_TUNE            980
#define VEHICLE_STOP            1000
#define VEHICLE_ABORT           2000    // Used to abort the current movement list and stop the robot

//======================================================================================
//======================================================================================
// Loads the command queue with the robots commands to be executed during a run
//======================================================================================
//======================================================================================
float x_target = 1000.0;
float y_target = 750.0;
float block_width = 1;
int current_direction = 1;  //1 = +y, 2 =+x, 3 = -y, 4 = -x



void loadCommandQueue() {

  cmdQueue.clear();
  cmdQueue.add(VEHICLE_START_WAIT);     // do not change this line - waits for start pushbutton
  cmdQueue.add(VEHICLE_START);          // do not change this line

  // Define robot movement speeds
  // Speed is encoder pulses per second.
  // There is a maximum speed.  Testing will be required to learn this speed.
  //    SETTING THE SPEEDS ABOVE THE MOTOR'S MAXIMUM SPEED WILL CAUSE STRANGE RESULTS
      cmdQueue.add(VEHICLE_SET_MOVE_SPEED,200);  // Ensure equal speed for both motors     // Speed used for forward movements  
      cmdQueue.add(VEHICLE_SET_TURN_SPEED,100);  // Adjusting to match forward speed     // Speed used for left or right turns
      cmdQueue.add(VEHICLE_SET_ACCEL,400);         // smaller is softer   larger is quicker and less accurate moves
        // Example list of robot movements
        // This block is modified for each tournament
        //ADD .15 BLOCK TO 1ST MOVE & SUBTRACT .15 BLOCK FROM LAST MOVE (compensate for offset of dowel to pivot point)
     
      //cmdQueue.add(VEHICLE_FORWARD,500);
      //cmdQueue.add(VEHICLE_REVERSE,500);
     // cmdQueue.add(VEHICLE_REVERSE,500);
    //  cmdQueue.add(VEHICLE_TURN_LEFT);
    //  cmdQueue.add(VEHICLE_TURN_RIGHT);
    //  cmdQueue.add(VEHICLE_TURN_180);

    //first move from start point, add 0.3 = 150mm estra so that the subsequest distances are mesured at ctr point of wheel axle
       cmdQueue.add(VEHICLE_FORWARD,500);

  cmdQueue.add(VEHICLE_TURN_RIGHT);
  cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_LEFT);
  // cmdQueue.add(VEHICLE_FORWARD,1000);

  // cmdQueue.add(VEHICLE_TURN_LEFT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_LEFT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_RIGHT);
  // cmdQueue.add(VEHICLE_FORWARD,500);
  
  // cmdQueue.add(VEHICLE_TURN_LEFT);

  // cmdQueue.add(VEHICLE_FORWARD,250);
  // cmdQueue.add(VEHICLE_REVERSE,250);
  
  // cmdQueue.add(VEHICLE_TURN_LEFT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_LEFT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_LEFT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_RIGHT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_FORWARD,250);
  // cmdQueue.add(VEHICLE_REVERSE,250);

  // cmdQueue.add(VEHICLE_TURN_RIGHT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_RIGHT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_LEFT);
  // cmdQueue.add(VEHICLE_FORWARD,500);

  // cmdQueue.add(VEHICLE_TURN_LEFT);
  // cmdQueue.add(VEHICLE_FORWARD,1000);

  // cmdQueue.add(VEHICLE_FORWARD,485);
  


  //subtract .15 = 75 mm from last

  //       // These  MUST be the last command in this orde.
  cmdQueue.add(VEHICLE_FINISHED);
    
      

}


void functionMode()
{
  switch (function_mode)
  {
  case IDLE:
    break;
  case IRREMOTE:
    break;
  case OBSTACLE:
    obstacleAvoidanceMode();
    break;
  case FOLLOW:
    followMode();
    break;
  case BLUETOOTH:
    break;
  case FOLLOW2:
    followMode2();
    break;
  default:
    break;
  }
}

void setMotionState()
{
  switch (motion_mode)
  {
  case FORWARD:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 20;
      setting_turn_speed = 0;
      break;
    case FOLLOW2:
      setting_car_speed = 20;
      setting_turn_speed = 0;
      break;
    case BLUETOOTH:
      setting_car_speed = 80;
      break;
    case IRREMOTE:
      setting_car_speed = 80;
      setting_turn_speed = 0;
      break;
    default:
      setting_car_speed = 40;
      setting_turn_speed = 0;
      break;
    }
    break;
  case BACKWARD:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = -20;
      setting_turn_speed = 0;
      break;
    case FOLLOW2:
      setting_car_speed = -20;
      setting_turn_speed = 0;
      break;
    case BLUETOOTH:
      setting_car_speed = -80;
      break;
    case IRREMOTE:
      setting_car_speed = -80;
      setting_turn_speed = 0;
      break;
    default:
      setting_car_speed = -40;
      setting_turn_speed = 0;
      break;
    }
    break;
  case TURNLEFT:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    case FOLLOW2:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    case BLUETOOTH:
      setting_turn_speed = 80;
      break;
    case IRREMOTE:
      setting_car_speed = 0;
      setting_turn_speed = 80;
      break;
    default:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    }
    break;
  case TURNRIGHT:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    case FOLLOW2:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    case BLUETOOTH:
      setting_turn_speed = -80;
      break;
    case IRREMOTE:
      setting_car_speed = 0;
      setting_turn_speed = -80;
      break;
    default:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    }
    break;
  case STANDBY:
    setting_car_speed = 0;
    setting_turn_speed = 0;
    break;
  case STOP:
    // Disabled: Balance check since hardware handles balancing
    // if (millis() - start_prev_time > 1000)
    // {
    //   function_mode = IDLE;
    //   if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
    //   {
    //     motion_mode = STANDBY;
    //     rgb.lightOff();
    //   }
    // }
    break;
  case START:
    // Disabled: Balance check since hardware handles balancing
    // if (millis() - start_prev_time > 2000)
    // {
    //   if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
    //   {
    //     car_speed_integeral = 0;
    //     setting_car_speed = 0;
    //     motion_mode = STANDBY;
    //     rgb.lightOff();
    //   }
    //   else
    //   {
    //     motion_mode = STOP;
    //     carStop();
    //     rgb.brightRedColor();
    //   }
    // }
    break;
  default:
    break;
  }
}

void keyEventHandle()
{
  if (key_value != '\0')
  {
    key_flag = key_value;

    switch (key_value)
    {
    case 's':
      rgb.lightOff();
      motion_mode = STANDBY;
      break;
    case 'f':
      rgb.flashBlueColorFront();
      motion_mode = FORWARD;
      break;
    case 'b':
      rgb.flashBlueColorback();
      motion_mode = BACKWARD;
      break;
    case 'l':
      rgb.flashBlueColorLeft();
      motion_mode = TURNLEFT;
      break;
    case 'i':
      rgb.flashBlueColorRight();
      motion_mode = TURNRIGHT;
      break;
    case '1':
      function_mode = FOLLOW;
      follow_flag = 0;
      follow_prev_time = millis();
      break;
    case '2':
      function_mode = OBSTACLE;
      obstacle_avoidance_flag = 0;
      obstacle_avoidance_prev_time = millis();
      break;
    case '3':
    rgb_loop:
      key_value = '\0';
      rgb.flag++;
      if (rgb.flag > 6)
      {
        rgb.flag = 1;
      }
      switch (rgb.flag)
      {
      case 0:
        break;
      case 1:
        if (rgb.theaterChaseRainbow(50) && key_value == '3')
          goto rgb_loop;
        break;
      case 2:
        if (rgb.rainbowCycle(20) && key_value == '3')
          goto rgb_loop;
        break;
      case 3:
        if (rgb.theaterChase(127, 127, 127, 50) && key_value == '3')
          goto rgb_loop;
        break;
      case 4:
        if (rgb.rainbow(20) && key_value == '3')
          goto rgb_loop;
        break;
      case 5:
        if (rgb.whiteOverRainbow(20, 30, 4) && key_value == '3')
          goto rgb_loop;
        break;
      case 6:
        if (rgb.rainbowFade2White(3, 50, 50) && key_value == '3')
          goto rgb_loop;
        break;
        break;
      default:
        break;
      }
      break;
    case '4':
      // Disabled: Balance reset since hardware handles balancing
      // function_mode = IDLE;
      // motion_mode = STOP;
      // carBack(110);
      // delay((kalmanfilter_angle - 30) * (kalmanfilter_angle - 30) / 8);
      // carStop();
      // start_prev_time = millis();
      // rgb.brightRedColor();
      break;
    case '5':
      // Disabled: Balance check since hardware handles balancing
      // if (millis() - start_prev_time > 500 && kalmanfilter_angle >= balance_angle_min)
      // {
      //   start_prev_time = millis();
      //   motion_mode = START;
      // }
      // motion_mode = START;
      break;
    case '6':
      rgb.brightness = 50;
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '7':
      rgb.brightRedColor();
      rgb.brightness -= 25;
      if (rgb.brightness <= 0)
      {
        rgb.brightness = 0;
      }
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '8':
      rgb.brightRedColor();
      rgb.brightness += 25;
      if (rgb.brightness >= 255)
      {
        rgb.brightness = 255;
      }
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '9':
      rgb.brightness = 0;
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '0':
      function_mode = FOLLOW2;
      follow_flag = 0;
      follow_prev_time = millis();
      break;
    case '*':
      break;
    case '#':
      break;
    default:
      break;
    }
    if (key_flag == key_value)
    {
      key_value = '\0';
    }
  }
}


void setup()
{

  Serial.begin(9600);
  //ultrasonicInit();
  keyInit();
  rgb.initialize();
  voltageInit();
  start_prev_time = millis();
  carInitialize();
  usLast = millis();
  loadCommandQueue();
  rgb.brightBlueColor();

}
bool moving = false;
void loop()
{
   
   //mpu6050.update();
  // Serial.print("angleX : ");
  // Serial.print(mpu6050.getAngleX());
  // Serial.print("\tangleY : ");
  // Serial.print(mpu6050.getAngleY());
  // Serial.print("\tangleZ : ");
  // Serial.println(mpu6050.getAngleZ());
  
  long distance;
  int speed;
  float fspd;
  long ldelta;
  long delayWait;
  int itmp;

    // this block calculates the number microseconds since this function's last execution
  unsigned long current = micros();
  usecElapsed = current - usLast;
  usLast = current;
  
     
  // update motor speed and status
  // mtrLeft.updateMotion(usecElapsed);
  // mtrRight.updateMotion(usecElapsed);
  extern bool isTrapezoidalMotion;
  if (!isTrapezoidalMotion) {
    updateStatus(usecElapsed);
  }

  
  int newCmd = false;
  if (cmdQueue.firstScan()) {
    newCmd = true;
    Serial.print(F("New Vehicle Cmd = "));
    Serial.println(cmdQueue.current());
  }

  int pbStart = !digitalRead(KEY_MODE);

  if (pbStart) {
   
    rgb.green(100);
    timerPBStartOn  += usecElapsed;
    timerPBStartOff = 0;
  } else {
    
    timerPBStartOn = 0;
    timerPBStartOff += usecElapsed;
  }
  
  if (cmdQueue.current() > VEHICLE_START && cmdQueue.current() < VEHICLE_ABORT) {
    if (timerPBStartOn > 100000) {
      cmdQueue.clear();
      cmdQueue.add(VEHICLE_ABORT);
      carStop();
      
    }
  }

  if (flagTimeRun) timerRunTime += usecElapsed;
  in_reverse = false;

  
  switch (cmdQueue.current()) {
    case VEHICLE_START_WAIT :
      if (timerPBStartOn > 100000) {
        cmdQueue.next();
      }
      timerSonicRange += usecElapsed;
      if (timerSonicRange > 1500000) {
        timerSonicRange = 0;
       // triggerRangeFinder();
      }
      break;
    case VEHICLE_START :
      timerRunTime = 0;
      motion_mode = STOP ;// START;  // Enable balance control
      if (timerPBStartOff > 100000) {
        cmdQueue.next();
        flagTimeRun = 1;
        flagLastMoveFwd = 0;
      }
      //mpu6050.update();
      //targetYaw = mpu6050.getGyroAngleZ();
      break;
      
    case VEHICLE_FORWARD :
      drive(1,newCmd);
      if (hasStopped) {
        delay(1000);  // Pause to allow balance stabilization before next command
        cmdQueue.next();
      }
      break;
    case VEHICLE_STRAIGHT :
      // stand_straight(newCmd);
      if (hasStopped) {
        // setMotorOutputs();
        
        // mtrRight.stop();
        // mtrLeft.stop();
        cmdQueue.next();
      }
      break;
    case VEHICLE_REVERSE :
      in_reverse = true;
       drive(-1,newCmd);
      if (hasStopped) {
        delay(1000);  // Pause to allow balance stabilization
        cmdQueue.next();
      }
      break;
    case VEHICLE_TURN_RIGHT :
      if (newCmd) {
        distance = ENCODER_COUNTS_90_DEG;
        speed = speedTurn;
        encoder_target = distance;
        turnRight();
      }
      if (hasStopped) {
        //setMotorOutputs();
        Serial.println("Turn completed.");
        delay(1000);  // Pause to allow balance stabilization
        cmdQueue.next();
      }      
      break;
    case VEHICLE_TURN_180 :  
      //turn(-180,newCmd);
      if (hasStopped) {
       // setMotorOutputs();
       delay(1000);  // Pause to allow balance stabilization
        cmdQueue.next();
      }      
      break;
    case VEHICLE_TURN_LEFT :
     if (newCmd) {
        carForwardTrapezoidal(-ENCODER_COUNTS_90_DEG, ENCODER_COUNTS_90_DEG, speedTurn);
      }
      
      if (hasStopped) {
        //setMotorOutputs();
        delay(1000);  // Pause to allow balance stabilization
        cmdQueue.next();
      }      
      break;
    case VEHICLE_SET_MOVE_SPEED :
      speedFwd = cmdQueue.getParameter1();
      cmdQueue.next();
      break;
    case VEHICLE_SET_TURN_SPEED :
      speedTurn = cmdQueue.getParameter1();
      cmdQueue.next();
      break;
    case VEHICLE_SET_ACCEL :
      //mtrLeft.setAccel(cmdQueue.getParameter1());
      //mtrRight.setAccel(cmdQueue.getParameter1());
      cmdQueue.next();
      break;
    default :
   
      break;
    case VEHICLE_FINISHED :
      if (newCmd) {
       // mtrLeft.stop();
       // mtrRight.stop();
       cmdQueue.clear();
       //carStop();
        //setMotorOutputs();
      }
      flagTimeRun = 0;
      break;
    case VEHICLE_ABORT :
      carStop();
      //setMotorOutputs();
      flagTimeRun = 0;
      if (timerPBStartOff > 200000) {
        loadCommandQueue();
      }
      break;
  }
      
  //******************************* 
  getKeyValue();
  getBluetoothData();
  keyEventHandle();
  getDistance();
//   voltageMeasure();
//   setMotionState();
//   functionMode();
//   checkObstacle();
//   rgb.blink(100);
//   static unsigned long print_time;
//   if (millis() - print_time > 100)
//   {
//     print_time = millis();
//     //Serial.println(kalmanfilter.angle);
//   }
//   static unsigned long start_time;
// Serial.println("st: ");
// Serial.println(pt);
// Serial.println("ct: ");
// Serial.println(millis());

//   if (millis() - pt > 5000)
//   {
//     function_mode = IDLE;
//     motion_mode = STOP;
//     carStop();
//   }else
//   {
//     Serial.println("movilg..");
//     carForward(40);
//   }
//   if (millis() - start_time == 2000) // Enter the pendulum, the car balances...
//   {
//     key_value = '5';
//   }
  
}

void drive(int direction, int newCmd) //1=fwd -1=rev
{
  if (newCmd) {
        // mpu6050.update();
        // target +=0;
        // noInterrupts();
        // reverse_last_enc = reverse_enc;
        // interrupts(); 
        // float distInmm = 0.0;
        // distInmm = (cmdQueue.getParameter1()-30); //40,correction due to break factor, -0.15 to compensentate the distnance of dowel from wheel axis
        // int distance = ((long) distInmm * (long) ENCODER_COUNTS_PER_REV / (long) MM_PER_REV);
        // targetPulses = ((long) distInmm * (long) ENCODER_COUNTS_PER_REV / (long) MM_PER_REV);
        // Serial.print("distaance: " + String(distance));
        // leftStart = mtrLeft.getEnc();
        // rightStart = mtrRight.getEnc();

        // rSpeed =csr.getSpeed(speedFwd);
        // lSpeed = csl.getSpeed(speedFwd);
        
        // mtrLeft.startMove(distance,direction*lSpeed);
        // mtrRight.startMove(distance,direction*rSpeed);
        // setMotorOutputs(); 

       encoder_target =  cmdQueue.getParameter1()*ENC_PER_MM;
       Serial.print(F("Distance requested (mm): "));
       Serial.print(cmdQueue.getParameter1());
       Serial.print(F(" | Encoder target: "));
       Serial.println(encoder_target);
       carForwardTrapezoidal(encoder_target,encoder_target,100);
        // adjust_target(direction * block_width);
      }else
      {
        if(hasStopped)
        {
          cmdQueue.next();
        }

        // if(usecElapsed < 1000)
        // {
        //   return;
        // }
        //long targetPulses = ((long) cmdQueue.getParameter1() * 500 * (long) ENCODER_COUNTS_PER_REV / (long) MM_PER_REV);;
      

        float Kp = 30, Ki = 2, Kd = 3;
        float error = 0, lastError = 0, integral = 0;
        int baseSpeed = speedFwd;

        int diff_enc = reverse_enc-reverse_last_enc;

        float distInmm = 0.0;
        distInmm = (cmdQueue.getParameter1()* block_width); //40,correction due to break factor, -0.15 to compensentate the distnance of dowel from wheel axis
        // int distance = ((long) distInmm * (long) ENCODER_COUNTS_PER_REV / (long) MM_PER_REV);
        
        // if(in_reverse && diff_enc >= distance-2)
        // {
        //   display.setCursor(0, 3);
        //   display.println("stopping:"  + String(diff_enc) + "    ");
        //     mtrRight.stop();
        //     mtrLeft.stop();
        //     setMotorOutputs();
        //     return;
        // }else
        // {
        //   // display.setCursor(0, 3);
        //   // display.println("running:" + String(diff_enc) + "    ");   
        // }

        // while ((mtrLeft.getEnc() - leftStart < targetPulses) &&
        //  (mtrRight.getEnc() - rightStart < targetPulses)) 
        // {
  //          mpu6050.update();
    //        float currentYaw = mpu6050.getAngleZ();

//            error = currentYaw - target;
          
            integral += error;
            integral = constrain(integral, -100, 100);
            float derivative = error - lastError;
            lastError = error;

            float correction = (Kp * error + Ki * integral + Kd * derivative)*direction;

            int leftSpeed = baseSpeed + correction;
            int rightSpeed = baseSpeed - correction;
           
            leftSpeed = constrain(leftSpeed, 200, 255);
            rightSpeed = constrain(rightSpeed, 200, 255); //150,200
            //moveForward(leftSpeed, rightSpeed);
            // analogWrite(PIN_MTR2_PWM,csr.getSpeed(rightSpeed));
            // analogWrite(PIN_MTR1_PWM,csl.getSpeed(leftSpeed));

            // Serial.print("Yaw: "); Serial.print(currentYaw);
            // Serial.print(" | Corr: "); Serial.print(correction);
            // Serial.print(" | Left: "); Serial.print(mtrLeft.getEnc() - leftStart);
            // Serial.print(" | Right: "); Serial.println(mtrRight.getEnc() - rightStart);
            
            //delay(50);
        // }
        // mtrLeft.stop();
        // mtrRight.stop();

      }


      //   //mpu6050.update();
      //   float current = mpu6050.getGyroAngleZ();
      //   float factor  = abs(read_yaw(current));
       
      //   display.setCursor(0,2);
      //   display.print(current);
      //   display.setCursor(10,2);
      //   display.print(target);

      //   int slow = 0;
      //   int fast = 0;
      //   //int factor = 0;
      //   if(round(current - target) > 0)
      //   {
      //     //drifiting left slow down right
      //     //prev_rightspeed = speedFwd;
          
      //     //factor = abs(current);
      //     lSpeed = constrain(lSpeed+factor,150,500);  
      //     rSpeed = constrain(rSpeed-factor,150,500);       

      //     analogWrite(PIN_MTR2_PWM,rSpeed);
      //     analogWrite(PIN_MTR1_PWM,lSpeed);
      //     display.setCursor(14,3);
      //     display.print(rSpeed);
          
      //   }
      //   if(round(target-current) > 0)
      //   {
        
      //     //factor = abs(current);
      //     lSpeed = constrain(lSpeed-factor,150,500);     
      //     rSpeed = constrain(rSpeed+factor,150,500);  

      //     //drifiting right slow down left
           
      //     // prev_leftspeed = constrain(prev_leftspeed-20,0,400);

      //     analogWrite(PIN_MTR2_PWM,rSpeed);
      //     analogWrite(PIN_MTR1_PWM,lSpeed);
         
      //   }
      //   // setMotorOutputs();
      // }
      // //display.clear();
      //   // display.setCursor(0,1);
      //   // display.print(prev_rightspeed);
      //   // display.setCursor(0,2);
      //   // display.print(prev_leftspeed);

}



