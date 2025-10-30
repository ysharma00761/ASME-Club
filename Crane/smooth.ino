// To Do:
// - Set up ARM switch
// - Set up ARM LED 
// - Set up Hook Motor
// - test rotation direction: if statements or constrain()?
// - set up and test stepright/stepleft buttons
// - Test and calibrate speeds
// - Consider third motor: move the hook further and closer to the tower
  // _ This requires a function that operates both steppers to keep the hook level as 
  // The carriage moves.
  // _ small rotary encoder for defining the height of the hook?
// - Consider using servos instead of steppers
// - Consider a bluetooth controller
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Stepper.h>

// ***** GENERAL MOTOR ASSIGNMENT *****
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution

// ***** PIN ASSIGNMENT *****
#define armSwitch 12
#define armedLED 13 // If using an UNO, you can use the onboard LED.
#define rotInput A0 // crane rotation motor input
#define hookInput A1 // hook motor input

// ***** MODE SELECT SETUP *****
bool ARMED = true; // If ARMED == true, the motors will recieve inputs.

// ***** ROTATION MOTOR SETUP *****
int rotRead = 512; // Initial pot reading is middle of range
int rotDir = 0; // Direction bit for motor rotation
int rotInputMin = 511; // lowest analog read
int rotInputMax = 512; // highest analog read
int rotMotor = 0; // The speed and direction of the rotation (positive is clockwise);
int rotSpeedLimit = 1000; // ticks per second max speed of rotation
const int rotDeadZone = 200; // rotation control dead zone
bool rotZeroPosition = false; // If rotation knob is in deadzone, this is TRUE
AccelStepper rotStepper(AccelStepper::FULL4WIRE, 8, 9, 10, 11); //originally a stepper object



// ***** HOOK MOTOR SETUP *****
int hookRead = 512; // Initial pot reading is middle of range
int hookDir = 0; // Direction for motor rotation
int hookInputMin = 511; // lowest analog read
int hookInputMax = 512; // highest analog read
int hookMotor = 0; // The speed and direction of the rotation (positive is clockwise);
int hookSpeedLimit = 1000; // ticks per second speed of rotation
const int hookDeadZone = 200; // rotation control dead zone
bool hookZeroPosition = false; // If hook knob is in deadzone, this is TRUE
AccelStepper hookStepper(AccelStepper::FULL4WIRE, 4, 5, 6, 7);



void setup() {
  Serial.begin(9600);
  pinMode(armSwitch, INPUT);
  pinMode(armedLED, OUTPUT);
  pinMode(rotInput, INPUT);

  hookStepper.setMaxSpeed(hookSpeedLimit);
  //hookStepper.setAcceleration(1800);
  rotStepper.setMaxSpeed(rotSpeedLimit);
  //rotStepper.setAcceleration(3000);

  ARMED = false;
}

void armCheck() {
  Serial.println(digitalRead(armSwitch));
  // Arming the machine requires 3 things:
    // 1) Arming Switch set High
    // 2) rotation control in zero position
    // 3) hook control in zero position
  ARMED = (digitalRead(armSwitch) == LOW) 
    && (rotZeroPosition)
    && (hookZeroPosition);
  // Dummy Declaration: remind operator to re-center potentiometers
  if ((digitalRead(armSwitch) == LOW) && (!rotZeroPosition || !hookZeroPosition)) {
    Serial.println("CANNOT ARM: SET CONTROLS TO DEADZONE!");
  }
}

void disarmCheck() {
  // Disarming the machine only requires the arming switch set LOW
  ARMED = !digitalRead(armSwitch);
}

void rotReadRoutine() {
  rotRead = analogRead(rotInput);
  if (!ARMED) {
    if (rotInputMin > rotRead) {rotInputMin = rotRead;} 
    if (rotInputMax < rotRead) {rotInputMax = rotRead;}
  }
  rotMotor = map(rotRead, rotInputMin, rotInputMax, -rotSpeedLimit, rotSpeedLimit);
  rotStepper.setSpeed(rotMotor);
  rotZeroPosition = abs(rotMotor) < rotDeadZone; 
  rotDir = constrain(rotMotor,-1,1);
  
  //Serial.print(rotInputMin); Serial.print(" - "); Serial.print(rotRead); Serial.print(" - "); Serial.println(rotInputMax);
}

void hookReadRoutine() {
  hookRead = analogRead(hookInput);
  if (!ARMED) {
    if (hookInputMin > hookRead) {hookInputMin = hookRead;}
    if (hookInputMax < hookRead) {hookInputMax = hookRead;}
  }
  hookMotor = map(hookRead, hookInputMin, hookInputMax, -hookSpeedLimit, hookSpeedLimit);
  hookStepper.setSpeed(hookMotor);
  hookZeroPosition = abs(hookMotor) < hookDeadZone;
  hookDir = constrain(hookMotor,-1,1);
  
  //Serial.print(hookInputMin); Serial.print(" - "); Serial.print(hookRead); Serial.print(" - "); Serial.println(hookInputMax);
}

void loop() {

    // ***** CALIBRATION MODE *****
      // While ARMED switch is toggled OFF
      // Continuous potentiometer calibration: 
      // Cycling a potentiometer from min to max will calibrate 
      // the potentiometer.
    while (!ARMED) {
      // Deactivate armedLED
      digitalWrite(armedLED, LOW);
      // Declare device ARM status
      Serial.println("***** NOT ARMED *****");
      rotReadRoutine();
      hookReadRoutine();
      armCheck();
      //delay(250); // Slow the serial output to be more readable.
      Serial.print("HOOK: "); Serial.print(hookZeroPosition); Serial.print(" - "); Serial.println(hookDir);
      Serial.print("ROTS: "); Serial.print(rotZeroPosition); Serial.print(" - "); Serial.println(rotDir);
    }

    // ***** OPERATING MODE ***** 
      // While ARMED switched is toggled ON
      // Continuously checks potentiometers and sends signals to motors
    while (ARMED) {

      digitalWrite(armedLED, HIGH);
      Serial.println("***** ARMED *****");

      // ***** ROTATION MOTOR *****
      rotReadRoutine();
      // Step the motor 4 steps
      if (!rotZeroPosition) {
        Serial.print("ROTS: "); Serial.println(rotDir);
        rotStepper.runSpeed();
      }
      // ***** HOOK MOTOR *****
      hookReadRoutine();
      // Step the motor 4 steps
      if (!hookZeroPosition) {
        Serial.print("HOOK: "); Serial.println(hookDir);
        hookStepper.runSpeed();
      }

      disarmCheck();
    }
    
  }
