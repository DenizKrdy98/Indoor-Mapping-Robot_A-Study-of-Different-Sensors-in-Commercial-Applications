// LIBS
#include <Wire.h>
#include <VL53L0X.h>      // laser sensor
#include "Ultrasonic.h"   // UVsensor
#include <SharpIR.h>      // IR sensor
#include <DFRobot_TFmini.h> // Lidar sensor
#include <I2Cdev.h>
#include <ADXL345.h>  // ADXL345 Accelerometer Library
#include <HMC5883L.h> // HMC5883L Magnetometer Library
#include <ITG3200.h>  // ITG3200 Gyroscope Library
#include <math.h>

// PIN CONFIG
#define IRPin A0
#define IRmodel 20150
#define UVPin 52
SoftwareSerial LidarSerial(13, 12); // RX, TX
SoftwareSerial Bluetooth(10, 11); // RX, TX

// SENSOR OBJECTS
DFRobot_TFmini  TFmini; /* Model : GP2Y0A02YK0F --> 20150 GP2Y0A21YK0F --> 1080 GP2Y0A710K0F --> 100500 GP2YA41SK0F --> 430 */
SharpIR mySensor = SharpIR(IRPin, IRmodel);   // Analog, 5V
Ultrasonic ultrasonic(UVPin);                 // UV: Digital Pin, 5V
VL53L0X laser;                                // LASER: SDA - SCL, 5V

// SENSOR DISTANCES
long UVdistance_cm;
long IRdistance_cm;
long LaserDistance_cm;
uint16_t LidarDistance,strength;

// SENSOR OFFSETS
long IROffset = 2.0;
long UVOffset = 2.5;
long LaserOffset = -3.0;
long LidarOffset = 0;

//---------------------- MOTORS AND DRIVER PARAMETERS
int MOTOR_A1 = 39; //brown
int MOTOR_A2 = 41; //red
int MOTOR_B1 = 43; //orange
int MOTOR_B2 = 45; //yellow
int MOTOR_C1 = 47; //green
int MOTOR_C2 = 49; //blue
int MOTOR_D1 = 51; //purple
int MOTOR_D2 = 53; //white

  /* dirs for goStraight() */
int UVdir = 1;
int LASERdir = 4;
int IRdir = 2;
int LIDARdir = 3;
  /* dirs for diagonalMove()*/
int UV_Las = 1;
int Las_Lid = 2;
int Lid_IR = 3;
int IR_UV = 4;
  /* dirs for wheel turns*/
int right = 1;
int left = 2;
int breaks = 3;
int released = 4;
//---------------------- MOTORS AND DRIVER PARAMETERS ENDS


String start = "start";
String stopp = "stop";
String command = "";


// Sensor Limits
  int UVmin = 5;
  int UVmin_stop = 22;
  int UVmax = 80;
  int IRmin = 20;
  int IRmin_stop = 22;
  int IRmax = 65;
  int LIDARmin = 30;
  int LIDARmin_stop = 30;
  int LIDARmax = 140;
  int LASERmin = 7;
  int LASERmin_stop = 18;
  int LASERmax = 80;

// Automated Movement Vars
  int dirToGo = UVdir;
  int goingDir = LASERdir;   // ********* USER DEFINED
  int dirSensorVal;
  int dirSensorMin;
  int eyeSensorVal;
  int eyeSensorMin;
  int keptEyeSensorVal;
  int directionState = 0;
  int loopCounter = 0;
  float toleranceDist = 6;  
  bool eyeSensorMinFixed = false;
  bool wallFound = false;
  bool justLeftTurned = false;
  int leftTurnWaiter = 0;
  int prevEyeSensorVal[] = {0,0,0,0,0};
  int prevDirSensorVal[] = {0,0,0,0,0};


// Bluetooth Waited Commands
void waitStartCommand(){   
   while(Bluetooth.available()== 0){}
   command = Bluetooth.readString();    
   command.trim();   
   while(!command.equalsIgnoreCase(start)){           
      while(Bluetooth.available()==0){}
      command = Bluetooth.readString();       
      command.trim();       
   }
}

bool hasStopCmdCome(){     
   command = Bluetooth.readString();    
   command.trim();   
   if(!command.equalsIgnoreCase(stop)){ 
      return true;
   }
   return false;
}

// *********************** PROGRAM SETUP ********************************
void setup()
{
  // MOTOR INITIALIZATION
  pinMode( MOTOR_A1, OUTPUT );
  pinMode( MOTOR_A2, OUTPUT );
  pinMode( MOTOR_B1, OUTPUT );
  pinMode( MOTOR_B2, OUTPUT );
  pinMode( MOTOR_C1, OUTPUT );
  pinMode( MOTOR_C2, OUTPUT );
  pinMode( MOTOR_D1, OUTPUT );
  pinMode( MOTOR_D2, OUTPUT );
  digitalWrite( MOTOR_A1, LOW );
  digitalWrite( MOTOR_A2, LOW );
  digitalWrite( MOTOR_B1, LOW );
  digitalWrite( MOTOR_B2, LOW );
  digitalWrite( MOTOR_C1, LOW );
  digitalWrite( MOTOR_C2, LOW );
  digitalWrite( MOTOR_D1, LOW );
  digitalWrite( MOTOR_D2, LOW );
  
  // SERIAL INITIALIZATIONS (Bluetooth, Lidar, Arduino Serial)
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);
  pinMode(13, INPUT);
  pinMode(12, OUTPUT);
  Serial.begin(9600);
  delay(200);
  Bluetooth.begin(9600);
  delay(200);
  waitStartCommand();
  delay(200);
  Wire.begin();
  TFmini.begin(LidarSerial);
    
  // LASER SETUP
  laser.setTimeout(500);
  if (!laser.init()){
    Bluetooth.println("Failed to detect and initialize laser!");
  }
  laser.startContinuous();
  
}


// ------------------ FINDING GOING DIR AND OBSERVATION DIR ----------
void updateDirectionAndEyeSensor(){
    if(goingDir == 1){
        if(UVdistance_cm == 2){        
          dirSensorVal = prevDirSensorVal[0];
          UVdistance_cm = dirSensorVal;
        }else{
           dirSensorVal = UVdistance_cm;
        }       
        dirSensorMin = UVmin_stop;
        eyeSensorVal = LaserDistance_cm;
        eyeSensorMin = LASERmin_stop;         
    }else if(goingDir == 2){
        dirSensorVal = IRdistance_cm;
        dirSensorMin = IRmin_stop;
        eyeSensorVal = LaserDistance_cm;
        eyeSensorMin = LASERmin_stop;
    }else if(goingDir == 3){
        dirSensorVal = LidarDistance;
        dirSensorMin = LIDARmin_stop;
        eyeSensorVal = LaserDistance_cm;
        eyeSensorMin = LASERmin_stop;
    }else if(goingDir == 4){
        dirSensorVal = LaserDistance_cm;
        dirSensorMin = LASERmin_stop;
        eyeSensorVal = IRdistance_cm;
        eyeSensorMin = IRmin_stop;
        
    }
}
// ------------------ FINDING GOING DIR AND OBSERVATION DIR ENDS ----------



// **** TURN FUNCS
void turn90(int LR){
  turnTo(LR);
  delay(685);
  
  if(LR == left){
  justLeftTurned = true;
    delay(43);
    directionState--;    
  }else if(LR == right){
    directionState++;
  }
  if(directionState == -1){
    directionState = 3;
  }
  directionState = directionState % 4;
  stopWith(released);
  delay(50);
  goStraightIn(dirToGo);
  delay(300);
}

bool LeftIsFree(int n){  
  int  prev = prevEyeSensorVal[n];
  if(justLeftTurned){
    leftTurnWaiter++;
  }
  if(leftTurnWaiter == 4){
    justLeftTurned = false;
    leftTurnWaiter = 0;
  }
  
  if(!justLeftTurned){
    if(eyeSensorVal > prev + 15){    
		return true;
	}
  }
  return false;
  
}

// **************** PROGRAM CYCLE ********************************
void loop(){ 
  sensorData();
  
  if(loopCounter>10){
      while(dirSensorVal>dirSensorMin){		
		if(hasStopCmdCome()){				// terminates the robot
			stopWith(breaks);
			while(true){ delay (10); } 		// inf loop			
		}
        goStraightIn(dirToGo);
        sensorData();		
        if(wallFound && LeftIsFree(1)){    // first degree check, if wall is found.          
          stopWith(breaks);
          sensorData();          
          if(LeftIsFree(2)){              // double check            
            goStraightIn(dirToGo);
            delay(200);
			sensorData();
            turn90(left);            
          }
        }
      }     
      stopWith(breaks);
      delay(500);
      turn90(right);
      wallFound = true;
  }
  
  loopCounter++;
}
// **************** PROGRAM CYCLE ENDS ********************************


// ------------------ SENSOR READ AND WRITE TO BLUETOOTH --------------------------
void sensorData(){
      
      // LASER CODE Read&Write      
      LaserDistance_cm = laser.readRangeContinuousMillimeters()/10.0 + LaserOffset;
      Bluetooth.print(LaserDistance_cm);      
      Bluetooth.print("\t");
      if (laser.timeoutOccurred()) { 
		Bluetooth.println('Laser Timeout!');
      }
      
      // UV CODE Read&Write
      UVdistance_cm = ultrasonic.MeasureInMillimeters()/10.0 + UVOffset; // two measurements should keep an interval
      if(UVdistance_cm == 2){
         Bluetooth.print(prevDirSensorVal[0]);//0~400cm
      }else{
         Bluetooth.print(UVdistance_cm);//0~400cm
      }
      Bluetooth.print("\t");
           
      
      // IR CODE Read&Write
      IRdistance_cm = mySensor.distance() + IROffset;
      Bluetooth.print(IRdistance_cm);
      Bluetooth.print("\t");

       // LIDAR CODE Read&Write
      if(TFmini.measure()){                                  
        LidarDistance = TFmini.getDistance() + LidarOffset;    
        Bluetooth.print(LidarDistance);
      }else{
        Bluetooth.print("999");
      }
  
    // STATE Read&Write
      Bluetooth.print("\t");
      Bluetooth.println(directionState+1);


      delay(200);

      updateDirectionAndEyeSensor();
      prevKeeper();
}
// ------------------ SENSOR READ AND WRITE TO BLUETOOTH ENDS  --------------------------

// ------------------ SENSOR DATA BUFFERS -----------------------------------------------
void prevKeeper(){  
   int temp[] = {0,0,0,0,0};
   for(int i=0;i<5;i++){
      temp[i] = prevEyeSensorVal[i];
   }
   prevEyeSensorVal[0] = eyeSensorVal;
   for(int i=0; i<4; i++){
      prevEyeSensorVal[i+1] = temp[i];
   }

   for(int i=0;i<5;i++){
      temp[i] = prevDirSensorVal[i];
   }
   prevDirSensorVal[0] = eyeSensorVal;
   for(int i=0; i<4; i++){
      prevDirSensorVal[i+1] = temp[i];
   }
}
// ------------------ SENSOR DATA BUFFERS ENDS ------------------------------------------


//---------------------- MOTORS AND DRIVER FUNCTIONS ---------------
void turnWithFrontWheels(int dir, int LR){
      
     if(dir==UVdir){
      motorA(left);
      motorB(left);
      motorC(right);
      motorD(right);
   }else if(dir==LASERdir){
      motorA(right);
      motorB(left);
      motorC(left);
      motorD(right);
   }else if(dir==LIDARdir){
      motorA(right);
      motorB(right);
      motorC(left);
      motorD(left);
   }else if(dir==IRdir){
      motorA(left);
      motorB(right);
      motorC(right);
      motorD(left);
   } 
}
  /* goes in the direction of selected sensor*/
void goStraightIn(int dir){
   if(dir==UVdir){
      motorA(left);
      motorB(left);
      motorC(right);
      motorD(right);
   }else if(dir==LASERdir){
      motorA(right);
      motorB(left);
      motorC(left);
      motorD(right);
   }else if(dir==LIDARdir){
      motorA(right);
      motorB(right);
      motorC(left);
      motorD(left);
   }else if(dir==IRdir){
      motorA(left);
      motorB(right);
      motorC(right);
      motorD(left);
   }  
}
  /* turns to the given direction*/
void turnTo(int dir){
      motorA(dir);
      motorB(dir);
      motorC(dir);
      motorD(dir);
}
  /* stops with break or releasing wheels*/
void stopWith(int choice){
      motorA(choice);
      motorB(choice);
      motorC(choice);
      motorD(choice);
}
  /* turns to the given direction*/
void diagonalMove(int dir){
   if(dir==UV_Las){
      motorA(released);
      motorB(left);
      motorC(released);
      motorD(right);
   }else if(dir==Las_Lid){
      motorA(right);
      motorB(released);
      motorC(left);
      motorD(released);
   }else if(dir==Lid_IR){
      motorA(released);
      motorB(right);
      motorC(released);
      motorD(left);
   }else if(dir==IR_UV){
      motorA(left);
      motorB(released);
      motorC(right);
      motorD(released);
   }
}
//---------------------- MOTORS AND DRIVER FUNCTIONS ENDS ---------------


//---------------------- MOTOR CONFIGURATIONS FOR DRIVER ---------------
void motorA(int dir) {
  if (dir == 1) {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
  } else if (dir == 2) {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
  } else if (dir == 3) {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, HIGH);
  } else {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
  }
}

void motorB(int dir) {
  if (dir == 1) {
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
  } else if (dir == 2) {
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
  } else if (dir == 3) {
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, HIGH);
  } else {
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
  }
}

void motorC(int dir) {
  if (dir == 1) {
    digitalWrite(MOTOR_C1, HIGH);
    digitalWrite(MOTOR_C2, LOW);
  } else if (dir == 2) {
    digitalWrite(MOTOR_C1, LOW);
    digitalWrite(MOTOR_C2, HIGH);
  } else if (dir == 3) {
    digitalWrite(MOTOR_C1, HIGH);
    digitalWrite(MOTOR_C2, HIGH);
  } else {
    digitalWrite(MOTOR_C1, LOW);
    digitalWrite(MOTOR_C2, LOW);
  }
}

void motorD(int dir) {
  if (dir == 1) {
    digitalWrite(MOTOR_D1, HIGH);
    digitalWrite(MOTOR_D2, LOW);
  } else if (dir == 2) {
    digitalWrite(MOTOR_D1, LOW);
    digitalWrite(MOTOR_D2, HIGH);
  } else if (dir == 3) {
    digitalWrite(MOTOR_D1, HIGH);
    digitalWrite(MOTOR_D2, HIGH);
  } else {
    digitalWrite(MOTOR_D1, LOW);
    digitalWrite(MOTOR_D2, LOW);
  }
}

//---------------------- MOTOR CONFIGURATIONS FOR DRIVER ENDS ---------------
