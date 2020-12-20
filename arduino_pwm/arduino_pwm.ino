#include <Servo.h>
#define POWER 0
#define STEERING 1

//************** MODE AND CONTROL VARS *****************//
char use_control_system = 0;
char mode = 0;

//************** CONTROL SYSTEM VARS ******************//
float speedInput = 0.0;     // meters/s
float kd = 100.0;
float kp = 200.0;
float ki = 20.0;
float integrator = 0.0;

char useControlSystem = 0;
float prevError = 0;
#define INTEGRATOR_CAP 150
#define TIRE_CIRCUMFERENCE 0.3 // meters
#define ENCODER_PULSES_PER_TIRE_REV 814 // This if for one revolution of the tire
#define PULSES_PER_METER 2713 // ENCODER_PULSES_PER_TIRE_REV/TIRE_CIRCUMFERENCE
#define POWER_MAX 1750
#define POWER_MIN 1450

//******************* SERIAL OBJECTS ******************//
String  inString = "";    // string to hold input
String  command = "";
String  valueString = "";
int commandStarted = 0;

float     value = 0;
long     baudRate = 115200;

//******************* SERVO OBJECTS ******************//
Servo steeringServo;
Servo powerServo;
int POWER_PIN = 5;
int STEERING_PIN = 9;
int PULSES = 1500; // Hard Right - 2000. Hard Left - 1000. Middle - 1500.
int PULSEP = 1600; // Fast Forward - 2000. Fast Reverse - 1000. Stopped - 1500.

//******************* ENCODER OBJECTS ******************//
#define encoder0PinA 2
#define encoder0PinB 3
#define updateEncoderPeriod 100 // value in ms
volatile signed long encoder0Pos = 0;
float encoderChange = 0;
signed long encoderLast = 0;
unsigned long startTime = 0;
unsigned long endTime = 0;

void setup()
{
  pinMode(STEERING_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  Serial.begin(baudRate);     // opens serial port, sets baud rate
  steeringServo.attach(STEERING_PIN);
  steeringServo.writeMicroseconds(PULSES);
  
  powerServo.attach(POWER_PIN);
  powerServo.writeMicroseconds(PULSEP);
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  //attachInterrupt(0, doEncoderA, RISING);

  startTime = millis();
  endTime = startTime+updateEncoderPeriod;

}

int checkForSerial(){
  //Function that checks if serial is available and parses commands
  if (Serial.available() > 0) {
      int inChar = Serial.read();
      if(inChar == '!'){
        commandStarted = 1;
        valueString = "";
        command = "";
      }

      if(commandStarted == 0){
        return 0;
      }
      if (isDigit(inChar) || inChar == '.') {
        valueString += (char)inChar;
      }if(isAlpha(inChar)){ 
        command += (char)inChar;
      }
      // if you get a newline, the command is complete.
      // update values if logical and send response.
      
      if (inChar == '\n') {
        commandStarted = 0;
        
        value = valueString.toInt();
        

        if(command=="p"){
          if(useControlSystem==0){
            if(value >=1000){
              if(value <= 2000){
                PULSEP = value;
                powerServo.writeMicroseconds(PULSEP);
                Serial.print("Power: ");
                Serial.println(PULSEP);
              }
            }
          }else{
            Serial.println("Cannot set power value directly when using control system.");
          }
        }else if(command=="s"){
          PULSES = value;
          steeringServo.writeMicroseconds(PULSES);
          Serial.print("Steering: ");
          Serial.println(PULSES);
          
        }else if(command=="enc"){
          //readEncoder(); 
        }
        else if(command=="usecs"){
          if(value==1){
            useControlSystem = 1;
            Serial.println("Using control system");
          }else if(value==0){
            useControlSystem = 0;
            Serial.println("Not using control system");
          }else{
            Serial.println("Value not acceptable for command: usecs");
          }
        }else if(command=="speedf"){
            if(value < 5){
              speedInput = value;
              //speedInput = speedInput/10.0;
              Serial.print("Forward Speed: ");
              Serial.println(value);
            }else{
              Serial.println("Value too high for command: speedf");
            }
          
        }else if(command=="speedr"){
            if(value < 5){
              speedInput = -value;
              //speedInput = speedInput/10.0;
              Serial.print("Reverse Speed: ");
              Serial.println(value);
            }else{
              Serial.println("Value too high for command: speedr");
            }
          
        }else{
        
          Serial.print("Command: (");
          Serial.print(command);
          Serial.println(") not recognized");
        }

        // clear the string for new input:
        valueString = "";
        command = "";
      }
    }  
    return 0;
}

/*void updateEncoder(){
  // function that will determine the frequency based on the time-lapse between encoder readings.
  //TODO setup interrupt code for two quadratures - so that it can determine direction.
  startTime = millis();
  if(startTime >= endTime){
    encoderChange = encoder0Pos-encoderLast;
    encoderLast = encoder0Pos;
    endTime = startTime+updateEncoderPeriod;
    Serial.println ("Hello, World!");
    if(useControlSystem==1){    
      controlSystem(speedInput);
    }
  }
}

void readEncoder(){
    Serial.print("E: ");
    Serial.println (encoder0Pos, DEC);
  
}

void controlSystem(float desired_speed){
  // function that will use the encoder frequency as the input to a PID control system
  // that will reach the desired speed.
  // encoder has ~814 counts for one revolution of the tire. Tire Circumference = 
  float actualSpeed = (encoderChange * (1000/updateEncoderPeriod))/PULSES_PER_METER;

  //Calculate proportional error
  float error = desired_speed - actualSpeed;
  integrator = integrator + error*ki;
  if(integrator > INTEGRATOR_CAP){
    integrator = INTEGRATOR_CAP;
  }else if(integrator < (-INTEGRATOR_CAP)){
    integrator = -INTEGRATOR_CAP;
  }
  Serial.print("INTEGRATOR: ");
  Serial.println(integrator);
  //Serial.print("ENCODER CHANGE: ");
  //Serial.println(encoderChange);
  float newPowerPulse = 1600 + error*kp + integrator;// -kd*(error-prevError);
  
  PULSEP = (int) newPowerPulse;
  if(PULSEP > POWER_MAX){
    PULSEP = POWER_MAX;
  }else if(PULSEP < POWER_MIN){
    PULSEP = POWER_MIN;
  }
  powerServo.writeMicroseconds(PULSEP);
  prevError = error;
  Serial.println(PULSEP);
  Serial.print("Actual Speed: ");
  Serial.println (actualSpeed, DEC);
}

void doEncoderA(){  
    // check channel B to see which way encoder is turning
    char value = PIND & 1<<3;
    //Serial.print("value: ");
    //Serial.println(value);
    if (value == 1<<3) {  
      encoder0Pos = encoder0Pos - 1;         // CCW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
  /*
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
    */

//}

void loop()
{
    checkForSerial();
    //updateEncoder();

}
