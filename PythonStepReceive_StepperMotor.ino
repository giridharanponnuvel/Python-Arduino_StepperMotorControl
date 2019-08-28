/*##################################################*/
//Author: Giridharan P
//Date: 16-03-2019
//Credits: Thanks to Asst.Prof Rajeev Lochana G C, Mechanical Department, Amrita School of Engineering, Bangalore &
//Prof Ganesha Udupa, Mechanical Department, Amrita School of Engineering, Amritapuri.
/*##################################################*/
//Purpose: This code is developed to get the step value from python and turn the stepper motors.
//Board used: Arduino Mega 2560
 
 //Motor 1, Motor 2 & Motor 3 connections
 
#define A_DIR_PIN          28 //PortA 6 | DigitalPin 28
#define A_STEP_PIN         26 //PortA 4 | DigitalPin 26
#define A_ENABLE_PIN       24 //PortA 2 | DigitalPin 24

#define B_DIR_PIN          38 //PortD 7 | DigitalPin 38
#define B_STEP_PIN         36 //PortC 1 | DigitalPin 36
#define B_ENABLE_PIN       34 //PortC 3 | DigitalPin 34

#define C_DIR_PIN          48 //PortL 1 | DigitalPin 48
#define C_STEP_PIN         46 //PortL 3 | DigitalPin 46
#define C_ENABLE_PIN       44 //PortL 5 | DigitalPin 44


//It is  faster  if the  ports  are  assigned  in bits. Changes are made only to the step pin port
#define A_STEP_HIGH             PORTA |=  0b00010000;
#define A_STEP_LOW              PORTA &= ~0b00010000;

#define B_STEP_HIGH             PORTC |=  0b00000010;
#define B_STEP_LOW              PORTC &= ~0b00000010;

#define C_STEP_HIGH             PORTL |=  0b00001000;
#define C_STEP_LOW              PORTL &= ~0b00001000;


//Intrupt is used to turn all the motors simultaenouly. At 1600 ticks(time), intrupt routine will check whether all the motors are moved or not.
// This intrupt routine helps us to divide the steps equaly and fed to the motor. So that it looks like all the stepper motors are moving simulatenouly. But if you watch high speed camera, you can see that each motor is moved one by one in a small steps (but equally divided) to appear that all the motor are moving simulatenouly.
#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];
String StringTheta1, StringTheta2, StringTheta3;

int StepTheta1 = 0;
int StepTheta2 = 0;
int StepTheta3 = 0;


struct stepperInfo {
  // externally defined parameters
  float acceleration;
  volatile unsigned long minStepInterval; // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)
  volatile unsigned long estStepsToSpeed;  // estimated steps required to reach max speed
  volatile unsigned long estTimeForMove;   // estimated time (interrupt ticks) required to complete movement
  volatile unsigned long rampUpStepTime;
  volatile float speedScale;               // used to slow down this motor to make coordinated movement with other motors

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  volatile unsigned int stepCount;         // number of steps completed in current movement
};


void aStep() {
  A_STEP_HIGH
  A_STEP_LOW
}
void aDir(int dir) {
  digitalWrite(A_DIR_PIN, dir);
}

void bStep() {
  B_STEP_HIGH
  B_STEP_LOW
}
void bDir(int dir) {
  digitalWrite(B_DIR_PIN, dir);
}

void cStep() {
  C_STEP_HIGH
  C_STEP_LOW
}
void cDir(int dir) {
  digitalWrite(C_DIR_PIN, dir);
}


void resetStepperInfo( stepperInfo& si ) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.rampUpStepTime = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}


#define NUM_STEPPERS 3


volatile stepperInfo steppers[NUM_STEPPERS];
void setup() {
  
  Serial.begin(9600); 
    //Clearing the StringTheta variable
  StringTheta1 = String();
  StringTheta2 = String();
  StringTheta3 = String();

  pinMode(A_STEP_PIN,   OUTPUT);
  pinMode(A_DIR_PIN,    OUTPUT);
  pinMode(A_ENABLE_PIN, OUTPUT);

  pinMode(B_STEP_PIN,   OUTPUT);
  pinMode(B_DIR_PIN,    OUTPUT);
  pinMode(B_ENABLE_PIN, OUTPUT);

  pinMode(C_STEP_PIN,   OUTPUT);
  pinMode(C_DIR_PIN,    OUTPUT);
  pinMode(C_ENABLE_PIN, OUTPUT);

  digitalWrite(A_ENABLE_PIN, LOW);
  digitalWrite(B_ENABLE_PIN, LOW);
  digitalWrite(C_ENABLE_PIN, LOW);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  interrupts();
  
  steppers[0].dirFunc = bDir;
  steppers[0].stepFunc = bStep;
  steppers[0].acceleration = 4000;
  steppers[0].minStepInterval = 50;

  steppers[1].dirFunc = aDir;
  steppers[1].stepFunc = aStep;
  steppers[1].acceleration = 4000;
  steppers[1].minStepInterval = 50;
  
  steppers[2].dirFunc = cDir;
  steppers[2].stepFunc = cStep; 
  steppers[2].acceleration = 4000;
  steppers[2].minStepInterval = 50;
}


//Below 2 functions will be called in DataCollector()
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '?';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();
            if (rc != endMarker) {
             receivedChars[ndx] = rc;
              ndx++;
               if (ndx >= numChars) {
                  ndx = numChars - 1;
               }
            }
            else {
            receivedChars[ndx] = 'e';
            ndx = 0;
            newData = true;
            }
        }
}

void showNewNumber() {
  byte recordFlag = 0;
	//The String should be like 20a30b40c? should be sent to arduino by python. Where 20 is the number of steps to be turned by motor A
	//30 is the no.of step to be turned vy motor B. Similarly 40 for motor C.
	//After receiving the string. The end marker ? is chaned into e
    if (newData == true) {
        for (byte i=0;receivedChars[i] != '?'; i++) {   
            
          if (receivedChars[i] == 'a'){
            recordFlag = 1;
            continue;
          }
          else if (receivedChars[i] == 'b'){
            recordFlag = 2;
            continue;
          }
          else if (receivedChars[i] == 'c'){
            recordFlag = 3;
            continue;
          }
          else if (receivedChars[i] == 'e'){
            //Send message to the arduino stating that all the charecters has been received.
            Serial.println("JD");
            break;
          }

          if (recordFlag == 1){
            StringTheta1+=receivedChars[i];
          }
          else if (recordFlag == 2){
            StringTheta2+=receivedChars[i];
          }
          else if (recordFlag == 3){
            StringTheta3+=receivedChars[i];
          }
        }

        // Convert String into integer:
        StepTheta1 = StringTheta1.toInt();
        StepTheta2 = StringTheta2.toInt();
        StepTheta3 = StringTheta3.toInt();
          
        newData = false;

       //Reset the StringTheta variables:
       StringTheta1 = String();
       StringTheta2 = String();
       StringTheta3 = String();      
    } 
}

void DataCollector() {
  recvWithEndMarker();
  showNewNumber();
}


void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
  si.speedScale = 1;

  float a = si.minStepInterval / (float)si.c0;
  a *= 0.676;

  float m = ((a*a - 1) / (-2 * a));
  float n = m * m;

  si.estStepsToSpeed = n;
}

volatile byte remainingSteppersFlag = 0;

float getDurationOfAcceleration(volatile stepperInfo& s, unsigned int numSteps) {
  float d = s.c0;
  float totalDuration = 0;
  for (unsigned int n = 1; n < numSteps; n++) {
    d = d - (2 * d) / (4 * n + 1);
    totalDuration += d;
  }
  return totalDuration;
}

void prepareMovement(int whichMotor, long steps) {
  volatile stepperInfo& si = steppers[whichMotor];
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  
  remainingSteppersFlag |= (1 << whichMotor);

  unsigned long stepsAbs = abs(steps);

  if ( (2 * si.estStepsToSpeed) < stepsAbs ) {
    // there will be a period of time at full speed
    unsigned long stepsAtFullSpeed = stepsAbs - 2 * si.estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si.estStepsToSpeed);
    si.estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * si.minStepInterval;
  }
  else {
    // will not reach full speed before needing to slow down again
    float accelDecelTime = getDurationOfAcceleration( si, stepsAbs / 2 );
    si.estTimeForMove = 2 * accelDecelTime;
  }
}

volatile byte nextStepperFlag = 0;

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned long mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    TIMER1_INTERRUPTS_OFF
    OCR1A = 65500;
  }

  OCR1A = mind;
}


ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      if ( s.stepCount >= s.totalSteps ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
      s.rampUpStepTime += s.d;
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d * s.speedScale; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}


void runAndWait() {
	//This function is used to rotate all the motor simulatenouly
  adjustSpeedScales();
  setNextInterruptInterval();
  TIMER1_INTERRUPTS_ON
  while ( remainingSteppersFlag );
  remainingSteppersFlag = 0;
  nextStepperFlag = 0;
}


void adjustSpeedScales() {
	//this function is used to accelerate the motor in a controlled manner.
  float maxTime = 0;
  
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;
    if ( steppers[i].estTimeForMove > maxTime )
      maxTime = steppers[i].estTimeForMove;
  }

  if ( maxTime != 0 ) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
      if ( ! ( (1 << i) & remainingSteppersFlag) )
        continue;
      steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
    }
  }
}


int FuncStepTheta(int motorName){
	//This function throws the number of steps to be moved by each motor separetely.
  if ( motorName == 0 ){
      return StepTheta1;
    }
  else if ( motorName == 1 ){
    return StepTheta2;
  }
  else if ( motorName == 2 ){
    return StepTheta3;
  }
  else {
    return 0;
  }
}


void loop() {
  //Calling data collector function
  DataCollector();
  
  //Preparing the stepper motor movement
   prepareMovement(0 ,FuncStepTheta(0));
   prepareMovement(1 ,FuncStepTheta(1));
   prepareMovement(2 ,FuncStepTheta(2));
   
   //runAndWait function does not work when the motor is at idle state.
  if (( StepTheta1 != 0 )&&(StepTheta2 != 0 )&&( StepTheta3 != 0 )){
  runAndWait();
  //Serial.println("JobDone");
  }
  
 /*
  Serial.println("StepTheta:");
  Serial.println(StepTheta1);
  Serial.println(StepTheta2);
  Serial.println(StepTheta3);
  */
  
  //Reseting the StepTheta variable. So that the motor will be in stop state after moving.
  StepTheta1 = 0;
  StepTheta2 = 0;
  StepTheta3 = 0;
}
