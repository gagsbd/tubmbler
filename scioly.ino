//========================================================================
// TopFinishKits.com Template Program for Robot Tour
//
//  Board: Arduino Uno
//  Vehicle: D4
//  Version: 2.1
//
// The information contained in this program is for general education 
// purposes only. The information is provided by TopFinishKits.com and 
// while we endeavor to keep the information up to date and correct, 
// we make no representations or warranties of any kind, express or 
// implied, about the completeness, accuracy, reliability, suitability 
// or availability with respect to the this program, or the website,
// information, products, services, or related graphics contained on
// the website for any purpose. Any reliance you place on such 
// information is therefore strictly at your own risk.
//
// Change Log:
//    2023-11-24  - Modified MotionLogic to only use minimum speed at the end
//                  of a move.
//                - Changed MotionLogic debug to work with the Serial Plotter.
//                - Minimum Speed is a constant set with the other constants.
//
//========================================================================
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>    // this library is needed for the 20x4 display
#include <MPU6050_tockn.h>
#include <CalibratedSpeed.h>
#include "Pins.h"

#define LeftMinSpeed 24
#define RightMinSpeed 24
#define LeftMaxSpeed 243
#define RightMaxSpeed 255

CalibratedSpeed csl = CalibratedSpeed(LeftMinSpeed, LeftMaxSpeed);
CalibratedSpeed csr = CalibratedSpeed(RightMinSpeed, RightMaxSpeed);

#define VERSION           "D4 2.1"

#define DISPLAY_PRESENT        1  // set to 1 if the 20x4 I2C Display is present


//
//  Board: Ardunio Uno
//  DEFINE ALL I/O PIN CONNECTIONS
//    *** DO NOT CHANGE ***
//
// #define PIN_MTR1_ENCA          2
// #define PIN_MTR2_ENCA          3
// #define PIN_PB_START           4
// #define PIN_MTR1_DIR_FWD       5
// #define PIN_MTR1_DIR_REV       6
// #define PIN_MTR2_DIR_FWD       7
// #define PIN_MTR2_DIR_REV       8
// #define PIN_MTR1_PWM           9
// #define PIN_MTR2_PWM          10
// #define PIN_SONIC_PULSE       11
// #define PIN_SONIC_TRIGGER     12
// #define PIN_LED               13

#define PIN_MTR1_ENCA ENCODER_LEFT_A_PIN
#define PIN_MTR2_ENCA ENCODER_RIGHT_A_PIN
#define PIN_PB_START KEY_MODE
#define PIN_MTR1_DIR AIN1
#define PIN_MTR2_DIR BIN1
#define PIN_MTR1_PWM PWMA_LEFT
#define PIN_MTR2_PWM PWMB_RIGHT
#define PIN_SONIC_TRIGGER TRIG_PIN
#define PIN_SONIC_PULSE ECHO_PIN
#define PIN_LED 13



//************************* ADJUST THE FOLLOWING TO MATCH YOUR ROBOT ****************************

#define ENCODER_COUNTS_PER_REV  540   // Set to the number of encoder pulses per wheel revolutiFon
#define MM_PER_REV              235   // Set to the number of mm per wheel revolution (Hence : Diameter * Pi)
#define ENCODER_COUNTS_90_DEG   333   // Set to the number of encoder pulses to make a 90 degree turn
#define SPEED_MIN               120    // Minimum speed (pulses/second) use at the end of individual moves

//ADDED
MPU6050 mpu6050(Wire,0.04,0.96);
float yaw;
float targetYaw;
float x=0;
float y=0;
// float x_target;
// float y_target;
// int current_direction = 1;
//int block_width = 500;

LiquidCrystal_I2C display(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

unsigned long usLast;
long usecElapsed;
long usScanLong;
int  usLongResetCount;
long usScanAvg;
long timerusScan;
int scanCount;

long timerRunTime;
int speedFwd = 10;
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
volatile uint8_t lastEncoderPins;

#define MAX_COMMANDS    60        // Maximum number of motion commands allowed

//======================================================================================
// Command object is used to hold a list of commands to be executed
//======================================================================================
class Command
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
  
    Command() {
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

Command cmdQueue;

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
float block_width = 1.0;
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
     // cmdQueue.add(VEHICLE_REVERSE,500);
     //cmdQueue.add(VEHICLE_TURN_LEFT);
      // cmdQueue.add(VEHICLE_TURN_RIGHT);
     //first move from start point, add 0.3 estra so that the subsequest distances are mesured at ctr point of wheel axle
      cmdQueue.add(VEHICLE_FORWARD,500);
      cmdQueue.add(VEHICLE_TURN_RIGHT);

      // cmdQueue.add(VEHICLE_FORWARD,500);
      // cmdQueue.add(VEHICLE_TURN_LEFT);

      // cmdQueue.add(VEHICLE_FORWARD,500);
      // cmdQueue.add(VEHICLE_TURN_LEFT);

      // cmdQueue.add(VEHICLE_FORWARD,500);
      // cmdQueue.add(VEHICLE_TURN_RIGHT);

      // cmdQueue.add(VEHICLE_FORWARD,485);

       //subtract .15 = 75 mm from last

  //       // These  MUST be the last command in this orde.
  cmdQueue.add(VEHICLE_FINISHED);

}

//======================================================================================
// Motion object (like a library) that calculates the acceleration used for motor speed
// control.
// 
// Distance is encoder pulses
// Speed is encoder pulses per second
// Accel is encoder pulses per second^2
//======================================================================================
class MotionLogic
{
  private:
    long timeAccel;
    long timeAtSpeed;
    long timeDecel;

    long timeRunning;
    long timeRunningLast;
    long timerUpdate;
    
    int pinPWM;
    int outputPWM;
    int outputFwd;
    int outputRev;

    int running;
    int flagStopped;
    int counterStopped;

    int pwmLoopI;
    int pwmLoopP; 

    int debugPrint;

  public:
    long accelRate;
    long decelRate;
    long position;
    long posProfile;

    int speedTarget;
    int speedProfile;
    int speedActual;
    int speedMinimum;

    int speedAtDecel;

    long countEncoder;
    long countEncoderLast;

    inline int getOutputPWM() { return outputPWM; };
    inline int getOutputFwd() { return outputFwd; };
    inline int getOutputRev() { return outputRev; };
    inline long getEnc() { return countEncoder; };


    inline void incrEncoder() { countEncoder++; };

    inline void debugOn() { debugPrint = 1; };   // used this function to turn on debug print statements
                                                 // recommended to only turn on debug for left or right motor.  NOT both.
    inline int debugState() { return debugPrint; };

    MotionLogic() {
      outputPWM = 0;
      outputFwd = 0;
      outputRev = 0;

      countEncoder = 0;
      countEncoderLast = 0;

      debugPrint = 0;

      accelRate = 200;
      decelRate = 200;

      posProfile = 0;

      speedActual = 0;
      speedTarget = 0;
      speedAtDecel = -10000;
      timeRunning = 0;
      timeRunningLast = 0;
      flagStopped = 0;
      counterStopped = 0;
      running = 0;
    }

    int isStopped() {
      if (flagStopped) return 1;
      return 0;
    }

    void setParams(long accel,int spdMin,int pPWM) {
      accelRate = accel;
      decelRate = accel;
      speedMinimum = spdMin;
      pinPWM = pPWM;
    }

    // This object uses the same value for accelerate and decelerate rate
    void setAccel(long accel) {
      accelRate = accel;
      decelRate = accel;
    }

    // This function must be called to start a motion
    void startMove(int pos, int spd) {

      if (spd > 0) {
        outputFwd = 1;
        outputRev = 0;
      } else {
        outputFwd = 0;
        outputRev = 1;
      }

      speedTarget = abs(spd);
      position = abs(pos);
      if (debugPrint) { Serial.print(F("Motion - speed       = ")); Serial.println(speedTarget); }
      if (debugPrint) { Serial.print(F("Motion - position    = ")); Serial.println(position); }

      float tAccel = (float) speedTarget / (float) accelRate;
      float tDecel = (float) speedTarget / (float) decelRate;
      float distAccel = (float) speedTarget / 2.0 * tAccel;
      float distDecel = (float) speedTarget / 2.0 * tDecel;
      float distAtSpeed = (float) position - distAccel - distDecel;
      if (distAtSpeed < 0.0) {  // current written as accel and decel same
        // Serial.println("Motion - Short move logic");
        distAccel = (float) position / 2.0;
        distDecel = (float) position / 2.0;
        distAtSpeed = 0.0;

        tAccel = sqrt(distAccel * 2.0 / (float) accelRate);
        tDecel = sqrt(distDecel * 2.0 / (float) decelRate);
      }
      float tAtSpeed = distAtSpeed / (float) speedTarget;

        // times are in microseconds
      timeAccel = 0;
      timeAtSpeed = tAccel * 1000000;
      timeDecel = timeAtSpeed + tAtSpeed * 1000000;


      pwmLoopI = 0;
      pwmLoopP = 0; 
      countEncoder = 0;
      countEncoderLast = 0;
      posProfile = 0;
      speedAtDecel = -10000;
      timeRunning = 0;
      timeRunningLast = 0;
      timerUpdate = 0;
      flagStopped = 0;
      counterStopped = 0;
      running = 1;
    }

    void stop() {
      outputPWM = 0;
      outputFwd = 0;
      outputRev = 0;
      analogWrite(pinPWM,outputPWM);
      running = 0;
      posProfile = 0;
      speedProfile = 0;
      speedTarget = 0;
      //ADDED
      flagStopped = 1;
    }

    void updateMotion(long usecElapsed) {
      int flagUpdate = 0;

      timerUpdate += usecElapsed;
      if (timerUpdate >= 30000) {
        long delta = countEncoder - countEncoderLast;
        
        speedActual = delta * 1000000L / timerUpdate;
        countEncoderLast = countEncoder;
        timerUpdate = 0;
        flagUpdate = 1;
        if (running == 0 && speedActual < 4) {
          if (!flagStopped) counterStopped++;
          if (counterStopped > 2) flagStopped = 1;
        }
      }

      if (running == 0) {
        outputPWM = 0;
        outputFwd = 0;
        outputRev = 0;
        pwmLoopP = 0;
        pwmLoopI = 0;
        return;
      }

//Serial.print("p: ");
//Serial.println(position);

//Serial.print("Encoder: ");
//Serial.println(countEncoder);
      if (countEncoder >= (position - 2)) {
        if (debugPrint) { 
          Serial.print("STOPPED,");
          Serial.print("timeRunning:");
          Serial.print(timeRunning);
          Serial.print(",encoder:");
          Serial.print(countEncoder);
          Serial.print(",speedActual:");
          Serial.print(speedActual);
          Serial.print(",pwm:");
          Serial.println(outputPWM);
        }

        stop();
        return;
      }

      timeRunning += usecElapsed;
      if (flagUpdate == 0) return;

      float speed;
      if (timeRunning < timeAtSpeed) {
        speed = (float) timeRunning / 1000000.0 * (float) accelRate;
        speedProfile = (int) speed;
        //if (debugPrint) { Serial.print(" - Accel speedProfile = "); Serial.println(speedProfile); }
      } else if (timeRunning < timeDecel) {
        speedProfile = speedTarget;
        //if (debugPrint) { Serial.print(" - At Speed speedProfile = "); Serial.println(speedProfile); }
      } else {
        if (speedAtDecel <= -10000) speedAtDecel = speedProfile;
        speed = (float) (timeRunning - timeDecel) / 1000000.0 * (float) decelRate;
        speedProfile = speedAtDecel - (int) speed;
        if (speedProfile < speedMinimum) speedProfile = speedMinimum;
        //if (debugPrint) { Serial.print(" - Decel speedProfile = "); Serial.println(speedProfile); }
      }

      posProfile += (speedProfile * (timeRunning - timeRunningLast)) / 1000000;
      timeRunningLast = timeRunning;

      long perror = posProfile - countEncoder;
      int serror = speedProfile - speedActual;
    
          // This program uses a PI loop to control speed
          // This loop is NOT tuned.  Meaning testing is required to tune the loop
          // which will provide the best preformance
      pwmLoopI += serror / 4;
      pwmLoopP = serror / 2; 
        
      outputPWM = pwmLoopP + pwmLoopI;
      if (outputPWM < 0) outputPWM = 0;
      if (outputPWM > 254) outputPWM = 254;

      if (debugPrint) { 
        Serial.print("timeRunning:");
        Serial.print(timeRunning);
        Serial.print(",encoder:");
        Serial.print(countEncoder);
        Serial.print(",speedProfile:");
        Serial.print(speedProfile);
        Serial.print(",speedActual:");
        Serial.print(speedActual);
        Serial.print(",pos_error:");
        Serial.print(perror);
        Serial.print(",speed_error:");
        Serial.print(serror);
        Serial.print(",loopI:");
        Serial.print(pwmLoopI);
        Serial.print(",loopP:");
        Serial.print(pwmLoopP);
        Serial.print(",pwm:");
        Serial.println(outputPWM);
      }

      analogWrite(pinPWM,outputPWM);
    }

};

MotionLogic mtrLeft;
MotionLogic mtrRight;

//---------------------------------------------------------------------------------------
// Interupt function for counting left motor encoder pulses
void encoderIntLeft()  { 
  mtrLeft.incrEncoder();
  
}

//---------------------------------------------------------------------------------------
// Interupt function for counting right motor encoder pulses
void encoderIntRight()  { 
  mtrRight.incrEncoder();
  
}

void setupEncoderInterrupts() {
  lastEncoderPins = PIND;
  PCICR |= _BV(PCIE2);            // enable pin-change interrupts for port D
  PCMSK2 |= _BV(PCINT18);         // PD2 / digital pin 2
  PCMSK2 |= _BV(PCINT20);         // PD4 / digital pin 4
}

ISR(PCINT2_vect) {
  uint8_t newPins = PIND;
  uint8_t changed = newPins ^ lastEncoderPins;

  if (changed & _BV(PD2)) {
    if (newPins & _BV(PD2)) encoderIntLeft();
  }
  if (changed & _BV(PD4)) {
    if (newPins & _BV(PD4)) encoderIntRight();
  }

  lastEncoderPins = newPins;
}

//---------------------------------------------------------------------------------------
void setMotorOutputs() {
  digitalWrite(PIN_MTR1_DIR, mtrLeft.getOutputFwd() ? LOW : HIGH);
  analogWrite(PIN_MTR1_PWM, mtrLeft.getOutputPWM());

  digitalWrite(PIN_MTR2_DIR, mtrRight.getOutputFwd() ? LOW : HIGH);
  analogWrite(PIN_MTR2_PWM, mtrRight.getOutputPWM());
}

//======================================================================================
// Trigger Sonic range finder
void triggerRangeFinder() {
    digitalWrite(PIN_SONIC_TRIGGER, LOW);
    delayMicroseconds(20);
    digitalWrite(PIN_SONIC_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONIC_TRIGGER, LOW);
    int pcount = pulseIn(PIN_SONIC_PULSE, HIGH);
    sonicDistance = float(pcount) * 0.34 / 2.0;
    //Serial.print(F("Sonic pcount = "));
    //Serial.print(pcount);
    //Serial.print(F("  mm = "));
    //Serial.println(sonicDistance);
}

//=======================================================================================
// Used to display fault codes on the built-in LED
void faultCodeLED(int count) {
  int i = -1;
  flagLED = true;
  toggleLED();
  while (-1) {
    i = count;
    delay(2000);

    while (i > 0) {
      toggleLED();
      delay(300);
      toggleLED();
      delay(300);
      i--;
    }
  }
}

//=======================================================================================
// Toggle the Ardunio built in LED each time this function is executed
void toggleLED() {
  if (flagLED) {                
    digitalWrite(PIN_LED,LOW);
    flagLED = false;
  } else {
    digitalWrite(PIN_LED,HIGH);
    flagLED = true;    
  }
}

//=======================================================================================
//  This function is called to update the variable information on the display.
//  Only limited information is updated at a time since the write commands are slow
//=======================================================================================
void initDisplay() {

  // Display is 20 characters wide by 4 lines

  // 01234567890123456789
  // Cmd:
  //   
  // Rng: xxxx.x cm
  // Time: xx.xxx  v.v.vv

  display.clear();
  display.setCursor(0,0);
  display.print(F("Cmd:"));

  display.setCursor(0,2);
 // display.print(F("Rng:"));

  display.setCursor(0,3);
  display.print(F("Time:"));

}

//=======================================================================================
//  This function is called to update the variable information on the display.
//  Only limited information is updated at a time since the write commands are slow
//=======================================================================================
void updateDisplay() {
  int cmd;
  char buff[12];
  int i;
  int itmp;
  float f;

  switch (printStep) {
    case 0 :
        cmd = cmdQueue.current();
        if (printLastCmd != cmd) {
          display.setCursor(4,0);
          switch (cmd) {
            case VEHICLE_START_WAIT :
                            // 4567890123456789
              display.print(F("WAIT START     "));
              break;
            case VEHICLE_START :
              display.print(F("WAIT RELEASE   "));
              break;
            case VEHICLE_FORWARD :
              display.print(F("FORWARD        "));
              break;
            case VEHICLE_REVERSE :
              display.print(F("REVERSE         "));
              break;
            case VEHICLE_TURN_RIGHT :
              display.print(F("TURN RIGHT     "));
              break;
            case VEHICLE_TURN_LEFT :
              display.print(F("TURN LEFT      "));
              break;
            case VEHICLE_FINISHED :
              display.print(F("FINISHED       "));
              break;
            case VEHICLE_STRAIGHT :
              display.print(F("STRAIGHT       "));
              break;
            case VEHICLE_TUNE :
              display.print(F("TUNE           "));
              break;
            case VEHICLE_STOP :
              display.print(F("STOP           "));
              break;
            case VEHICLE_ABORT :
              display.print(F("ABORT          "));
              break;
            default :
              display.print(F("***unknown**"));
              break;            
          }
          printLastCmd = cmd;
        }
        break;
    case 1 :
        break;
    case 2 :
        break;
    case 3 :
        display.setCursor(4,2);
        display.print(sonicDistance,1);
        //display.print("cm  ");
        break;
    case 4 :
        display.setCursor(6,3);
        f = (float) timerRunTime / 1000000.0; 
        display.print(f,3);
        break;
    default :
        printStep = -1;
        break;
  }

  printStep++;  // increment the print step value to the next sequence step
  
  toggleLED();
}

//ADDED
int lSpeed = speedFwd;
int rSpeed = speedFwd;

//variables for PID control
float target = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;



//======================================================================================
// The setup() is called once at the power up of the Arduino
//======================================================================================
void setup() {

//ADDED
  Wire.begin();
 
  pinMode(PIN_LED, OUTPUT);
  flagLED = false;
  
  
  // Only uncomment one motor at a time to use the Serial Plotter function to tune the PID loop
  //mtrLeft.debugOn();
  //mtrRight.debugOn();
Serial.begin(115200);
  if (mtrLeft.debugState() || mtrRight.debugState()) {
    Serial.begin(115200);
    Serial.println(F("Setup()..."));
  }
 

  if (DISPLAY_PRESENT) {
    //Serial.println(F("Display init()"));
    display.init();  //initialize the lcd
    display.backlight();  //open the backlight 
    display.clear();
    display.setCursor(0,0);
    display.print(F("Start Up....."));
  }

  pinMode(PIN_SONIC_TRIGGER,OUTPUT);
  pinMode(PIN_SONIC_PULSE,INPUT);

  pinMode(PIN_PB_START, INPUT_PULLUP);

  pinMode(PIN_MTR1_PWM,OUTPUT);
  pinMode(PIN_MTR2_PWM,OUTPUT);

  pinMode(PIN_MTR1_ENCA, INPUT_PULLUP);
  pinMode(PIN_MTR2_ENCA, INPUT_PULLUP);

  sonicDistance = 0.0;
  timerSonicRange = 0;

  timerDelay = 0;

  setupEncoderInterrupts();

  pinMode(PIN_MTR1_DIR,OUTPUT);
  pinMode(PIN_MTR2_DIR,OUTPUT);
  digitalWrite(PIN_MTR1_DIR, LOW);
  digitalWrite(PIN_MTR2_DIR, LOW);

  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

speedFwd = 200;   //changed
  speedTurn = 100;
  
  //ADDED
  

  int speedAccel = 100;
  int speedMin  = SPEED_MIN;
  mtrLeft.setParams(speedAccel, speedMin, PIN_MTR1_PWM);
  mtrRight.setParams(speedAccel, speedMin, PIN_MTR2_PWM);

  //Serial.println(F("Initialize Display with background text"));
  if (DISPLAY_PRESENT) { initDisplay(); }
  printStep = 0;

  loadCommandQueue();
  usScanLong = 0;
  usScanAvg = 0;
  timerusScan = 0;
  scanCount = 0;
  usLongResetCount = 0;
  //Serial.println(F("....End Setup"));

  usLast = micros();

//ADDED
//Serial.println("cal mpu");
mpu6050.begin();
mpu6050.calcGyroOffsets(true);

//mpu.CalibrateAccel(20);
//mpu.CalibrateGyro(20);
//delay(200);
  // mpu.setXAccelOffset(0); //Set your accelerometer offset for axis X
  // mpu.setYAccelOffset(0); //Set your accelerometer offset for axis Y
  // mpu.setZAccelOffset(0); //Set your accelerometer offset for axis Z
  // mpu.setXGyroOffset(0);  //Set your gyro offset for axis X
  // mpu.setYGyroOffset(0);  //Set your gyro offset for axis Y
  // mpu.setZGyroOffset(0);  //Set your gyro offset for axis Z

}



//======================================================================================
// The following function will execute then exit.  The Ardunio will constantly call this 
// function.  The function should not have delays as this will effect the motor's speed.
//======================================================================================
void loop() {

   mpu6050.update();
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
  if (usecElapsed > usScanLong) usScanLong = usecElapsed;
  timerusScan += usecElapsed;
  scanCount++;
  if (timerusScan > 1000000) {
    usScanAvg = timerusScan / scanCount;
    timerusScan = 0;
    scanCount = 0;
    usLongResetCount++;
    if (usLongResetCount > 10) {
      usScanLong = 0;
      usLongResetCount = 0;
    }
  }

 
     
  // update motor speed and status
  mtrLeft.updateMotion(usecElapsed);
  mtrRight.updateMotion(usecElapsed);

    // updates the display every 200000us or 0.2 seconds
  if (msTimerPrint > 200000) {
    if (DISPLAY_PRESENT) { updateDisplay(); }
    msTimerPrint = 0;
  }
  msTimerPrint += usecElapsed;



  int newCmd = false;
  if (cmdQueue.firstScan()) {
    newCmd = true;
    Serial.print(F("New Vehicle Cmd = "));
    Serial.println(cmdQueue.current());
  }

  int pbStart = !digitalRead(PIN_PB_START);

  if (pbStart) {
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
      mtrLeft.stop();
      mtrRight.stop();
      setMotorOutputs();
    }
  }

  if (flagTimeRun) timerRunTime += usecElapsed;
  
  switch (cmdQueue.current()) {
    case VEHICLE_START_WAIT :
      if (timerPBStartOn > 100000) {
        cmdQueue.next();
      }
      timerSonicRange += usecElapsed;
      if (timerSonicRange > 1500000) {
        timerSonicRange = 0;
        triggerRangeFinder();
      }
      break;
    case VEHICLE_START :
      timerRunTime = 0;
      if (timerPBStartOff > 100000) {
        cmdQueue.next();
        flagTimeRun = 1;
        flagLastMoveFwd = 0;
      }
      mpu6050.update();
      targetYaw = mpu6050.getGyroAngleZ();
      break;
      
    case VEHICLE_FORWARD :
      drive(1,newCmd);
      if (mtrLeft.isStopped() && mtrRight.isStopped()) {
        mtrRight.stop();
        mtrLeft.stop();
        cmdQueue.next();
      }
      break;
    case VEHICLE_STRAIGHT :
      // stand_straight(newCmd);
      if (mtrLeft.isStopped() && mtrRight.isStopped()) {
        mtrRight.stop();
        mtrLeft.stop();
        cmdQueue.next();
      }
      break;
    case VEHICLE_REVERSE :
      
       drive(-1,newCmd);
      if (mtrLeft.isStopped() && mtrRight.isStopped()) {
        mtrRight.stop();
        mtrLeft.stop();
        cmdQueue.next();
      }
      break;
    case VEHICLE_TURN_RIGHT :
      turn(-87,newCmd);
      if (mtrLeft.isStopped() && mtrRight.isStopped()) {
        setMotorOutputs();
        cmdQueue.next();
      }      
      break;
    case VEHICLE_TURN_LEFT :
      turn(86,newCmd);
      if (mtrLeft.isStopped() && mtrRight.isStopped()) {
        setMotorOutputs();
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
      mtrLeft.setAccel(cmdQueue.getParameter1());
      mtrRight.setAccel(cmdQueue.getParameter1());
      cmdQueue.next();
      break;
    default :
   
      break;
    case VEHICLE_FINISHED :
      if (newCmd) {
        mtrLeft.stop();
        mtrRight.stop();
       
        //setMotorOutputs();
      }
      flagTimeRun = 0;
      break;
    case VEHICLE_ABORT :
      mtrLeft.stop();
      mtrRight.stop();
      setMotorOutputs();
      flagTimeRun = 0;
      if (timerPBStartOff > 200000) {
        loadCommandQueue();
      }
      break;
  }

}

void turn(float delta,int newCmd)
{
      // delta positive = left turn, negative = right turn
      if (delta == 0.0f) return;
      int direction = (delta > 0.0f) ? 1 : -1;

      if (newCmd) {
        float current = mpu6050.getGyroAngleZ();
        targetYaw = current + delta;
        while (targetYaw > 180.0f) targetYaw -= 360.0f;
        while (targetYaw <= -180.0f) targetYaw += 360.0f;

        Serial.print("Turn start currentYaw = ");
        Serial.print(current);
        Serial.print(" targetYaw = ");
        Serial.println(targetYaw);

        int distance = ENCODER_COUNTS_90_DEG * 20; // sufficient encoder limit, MPU controls stop
        mtrLeft.startMove(distance, speedTurn * -direction);
        mtrRight.startMove(distance, speedTurn * direction);
        setMotorOutputs();

        display.setCursor(0,0);
        display.print("Turning            ");
        display.setCursor(0,1);
        display.print("Target angle:" + String(targetYaw) + "  ");
        Serial.println("Turning");
      } else {
        float current = mpu6050.getGyroAngleZ();
        float error = targetYaw - current;
        while (error > 180.0f) error -= 360.0f;
        while (error <= -180.0f) error += 360.0f;

        if (fabs(error) > 2.0f) {
           display.setCursor(0,2);
           display.print("Current angle:" + String(current) + "  ");
        } else {
          mtrLeft.stop();
          mtrRight.stop();
          setMotorOutputs();
        }
      }
}

volatile long leftTotalCount = 0;
volatile long rightTotalCount = 0;
long targetPulses = 0;
long leftStart = 0;
long rightStart = 0;

void drive(int direction, int newCmd) //1=fwd -1=rev
{
  if (newCmd) {
        // mpu6050.update();
        target +=0;
        float distInmm = 0.0;
        distInmm = ((cmdQueue.getParameter1())* block_width); //40,correction due to break factor, -0.15 to compensentate the distnance of dowel from wheel axis
        int distance = ((long) distInmm * (long) ENCODER_COUNTS_PER_REV / (long) MM_PER_REV);
        targetPulses = ((long) distInmm * (long) ENCODER_COUNTS_PER_REV / (long) MM_PER_REV);
        Serial.print("distaance: " + String(distance));
        leftStart = mtrLeft.getEnc();
        rightStart = mtrRight.getEnc();

        rSpeed =csr.getSpeed(speedFwd);
        lSpeed = csl.getSpeed(speedFwd);

        mtrLeft.startMove(distance,direction*speedFwd);
        mtrRight.startMove(distance,direction*speedFwd);
        setMotorOutputs(); 
        display.setCursor(0,1);
        display.print("Trget angle:" + String(target) + "  ");
        // adjust_target(direction * block_width);
      } else {
        // Let the motion objects update the motor outputs in the main loop.
        // Do not block here with a raw analogWrite loop.
        return;
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

float read_yaw(float current)
{
  //the 'k' values are the ones you need to fine tune before your program will work. Note that these are arbitrary values that you just need to experiment with one at a time.
  float Kp = 10;
  float Ki = 0.1;
  float Kd = 5;
    //mpu6050.update();
    //float current  = mpu6050.getGyroAngleZ();

    error = target - current;// proportional
    integral = integral + error; //integral
    derivative = error - last_error; //derivative

    float result = (error * Kp) + (integral * Ki) + (derivative * Kd);
    last_error = error;
    return result;

}
// void adjust_direction(int left)
// {
//     switch (current_direction){
//     case 1 :  //+y
//        if(left)
//        {
//           current_direction = 4;
//        }else{
//           current_direction = 2;
//        }
//        break;
//     case 2 :  //+x
//        if(left)
//        {
//           current_direction = 1;
//        }else{
//           current_direction = 3;
//        }
//        break;
//     case 3 :  //-y
//       if(left)
//        {
//           current_direction = 2;
//        }else{
//           current_direction = 4;
//        }
//        break;
//     case 4 : //-x
//        if(left)
//        {
//           current_direction = 3;
//        }else{
//           current_direction = 1;
//        }
//        break; 
//    }
// }
// void adjust_target(float length)
// {
//    switch (current_direction){
//     case 1 : //+y
//        y += length;
//        break;
//     case 2 : //+x
//        x += length;
//        break;
//     case 3 :  //-y
//       y -= length;
//       break;
//     case 4 :  //x
//       x -= length;
//       break;  
//    }
   
// }

// void stand_straight(int newCmd)
// {
//   if(!newCmd)
//   {
//     return;
//   }
//   switch (current_direction){
//     case 1 : //+y
       
//        break;
//     case 2 : //+x
//        turn(90,newCmd);
//        break;
//     case 3 :  //-y
//       turn(180,newCmd);
//       break;
//     case 4 :  //x
//       turn(-90,newCmd);
//       break;  
//    } 
// }

// void tune(int newCmd)
// {
//   if( !newCmd)
//   {
//     return;
//   }

//   float angle_radians = atan2(y-y_target, x-x_target);
//   float angle_degrees = degrees(angle_radians);
//   if(angle_degrees < 0)
//   {
//     turn(90+angle_degrees);
//   }else
  

// }