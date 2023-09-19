/*
   How to configure and pair two HC-05 Bluetooth Modules
   by Dejan Nedelkovski, www.HowToMechatronics.com

                   == SLAVE CODE ==
*/

#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

#include <ArduinoJson.h>
//#include <SoftwareSerial.h>
#define button 8
//SoftwareSerial myserial(7,8);
Servo base;
Servo elbo;
Servo shoulder;
Servo palm;
Servo finger;
int state = 20;
int buttonState = 0;
int k, l, m;
int motion;

//int shoulderAngle=1400,baseAngle=1400,palmAngle=1400,elboAngle=1400,fingerAngle=1400;

int Ax, Ay, Az,pot_fin,Gz,pot_z,pick_ang=1500,palm_ang;

#include <Servo.h>
// ramp lib
#include <Ramp.h>

int servo1Offset = 700;      // gripper closed - max 2200
int servo2Offset = 1400;      // shoulder back - min 600  (1120 mid)
int servo3Offset = 1400;      // elbow back - max 2180 (1180 mid)
int servo4Offset = 1555;
// moving the foot forwards or backwards in the side plane
double shoulderAngle2;
double shoulderAngle2a;
double shoulderAngle2Degrees;
double shoulderAngle2aDegrees;
double z2;
int x = 100;

// side plane of individual leg only
double lowerLength = 280.0;
double upperLength = 180.0;
double armLength;
int z = 320;
double b = 600;
double shoulderAngle;
double shoulderAngleDegrees;
double shoulderAngle1a;
double elbowAngle;
double elbowAngleDegrees;

// interpolation targets
double zTarget;
double xTarget;

// output scaling
double shoulderMs;
double shoulderMs2;
double elbowMs;
bool flag=0;

// wheel encoder interrupts

volatile long encoder0Pos = 0;    // encoder 1
long encoder0Target = 0;

unsigned long currentMillis;
//unsigned long previousMillis=millis();

long previousStepMillis = 0;    // set up timers


class Interpolation {
  public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue = 10;

    int go(int input, int duration) {

      if (input != savedValue) {   // check for new data
        interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value

      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
        myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
        interpolationFlag = 1;
      }

      int output = myRamp.update();
      return output;
    }
};    // end of class

Interpolation interpX;
Interpolation interpZ;
 unsigned long y_range;





void setup() {
  pinMode(button, INPUT);
  base.attach(5);

  shoulder.attach(6);
  elbo.attach(9);

  palm.attach(10);
  finger.attach(11);
  Serial.begin(9600); // Default communication rate of the Bluetooth module
  //  myserial.begin(9600);

  shoulder.writeMicroseconds(1350);
//  base.writeMicroseconds(1400);
//  elbo.writeMicroseconds(servo3Offset);
//  palm.writeMicroseconds(1400);
//  finger.writeMicroseconds(1600);

    lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("ROBOTIC ARM");
  lcd.setCursor(2,1);
  lcd.print("HI");
  
}

void loop() {
  if (Serial.available() > 0) { // Checks whether data is comming from the serial port
    //    state = myserial.read(); // Reads the data from the serial port
    //    Serial.print("pot value  :");
    //    Serial.println(state);

    StaticJsonDocument<600> doc;

    DeserializationError err = deserializeJson(doc, Serial);

    if (err == DeserializationError::Ok)
    {
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)

      Ax = doc["Ax"].as<long>();

      Ay = doc["Ay"].as<int>();

      pot_fin = doc["pot_fin"].as<int>();
      pot_z = doc["pot_z"].as<int>();
      Gz = doc["Gz"].as<int>();

//      if(millis()-previousMillis>500)
//      {
//        previousMillis=millis();
//      lcd.clear();
//      lcd.setCursor(0,1);
//      lcd.print(Ax);
//      lcd.setCursor(4,1);
//      lcd.print(Ay);
//      lcd.setCursor(8,1);
//      lcd.print(pot_fin);
//      lcd.setCursor(0,0);
//      lcd.print(z);
//      lcd.setCursor(6,0);
//      lcd.print(x);
//      lcd.setCursor(12,0);
//      lcd.print(y_range);
      flag=1;
//      }

      //      m=doc["m"].as<int>();
    }
    else 
    {
      // Print error to the "debug" serial port
//       lcd.clear();
//      lcd.setCursor(0,0);
//      lcd.print("deserializeJson() returned ");
//      lcd.setCursor(0,1);
//      lcd.print(err.c_str());
      flag=0;
      // Flush all bytes in the "link" serial port buffer
    
    }


  }
  else
  {
    flag=0;
  }




   y_range = sqrt(sq(45) - sq(z/10))*10;
   



  
  


  //   Serial.print(Ax);
  //
  // Serial.print("    ");
  //
  //  Serial.println(Ay);
  
  if (Ax < -50)
  {
    x++;
    x=constrain(x,0, y_range);
    
  }
  if (Ax > 50)
  {
    x--;
    x=constrain(x,0, y_range);
  }
  if (Ay < -50)
  {
    b+=5;
    b=constrain(b, 600,2100);
  
  }
  if (Ay > 50)
  {
    b-=5;
   b= constrain(b, 600, 2100);
  }
  ///////////////


//  if (pot_z <250)
//  {
//    z--;
//    z=constrain(z,60,400);
//  }
//    if (pot_z >250 && pot_z <500)
//  {
//    z++;
//    z=constrain(z,60,400);
//  }
//     if (pot_z >500)
//  {
//    z=z;
//  }
  ////////////////
    if (pot_fin >600)
  {
    pick_ang+=50;
    pick_ang=constrain(pick_ang,1500,2300);
  }
    if (pot_fin <600 && pot_fin >350)
  {
    pick_ang-=50;
    pick_ang=constrain(pick_ang,1500,2300);
  }
     if (pot_fin <350)
  {
    pick_ang=pick_ang;
  }
 //////
// if(Gz<-50)
// {
//  palm_ang+=100;
//  palm_ang=constrain(palm_ang,600,1800);
// }
//  if(Gz>50)
// {
//  palm_ang-=100;
//  palm_ang=constrain(palm_ang,600,1800);
// }
  




  z2 = sqrt(sq(x) + sq(z));   // calc new arm length to feed to the next bit of code below
  //          Serial.print(" z2 =");
  //  Serial.print(z2);
  //  // ****************
  //  Serial.print("    ");

  // calculate arm length based on upper/lower length and elbow and shoulder angle
  shoulderAngle2a = (sq(x) + sq(z2) - sq(z)) / (2 * x * z2);
  double shi = acos(shoulderAngle2a);
  //Serial.print(shi);
  //Serial.print("    ");
  shoulderAngle1a = (sq(lowerLength) + sq(z2) - sq(upperLength)) / (2 * lowerLength * z2);
  // Serial.print(shoulderAngle1a);
  // Serial.print("    ");

  double theeta = acos(shoulderAngle1a);     // radians
  // Serial.print(theeta);


  shoulderAngle = theeta + shi;

  elbowAngle =  (sq(lowerLength)  + sq(upperLength) - sq(z2)) / (2 * lowerLength * upperLength );   // radians
  double alpha = acos(elbowAngle);
  // Serial.println(elbowAngle);
  elbowAngleDegrees = alpha * (180 / PI);
  //  Serial.print(elbowAngleDegrees);
  //  Serial.print("    ");
  shoulderAngleDegrees = shoulderAngle * (180 / PI);  // degrees
  //  Serial.print(shoulderAngleDegrees);

  shoulderMs = ((180 - shoulderAngleDegrees) * 10.0 ) + 600;
  elbowMs = ((180 - elbowAngleDegrees) * 9.0  ) + 600;

  //  Serial.print("    ");
  //  Serial.print(shoulderAngleDegrees);
  //  Serial.print("    ");
  //  Serial.print(180-elbowAngleDegrees);
  //  Serial.println("    ");
  //
  ////
//  float ang = interpX.go(shoulderMs, 1000);
//  float ang2 = interpZ.go(elbowMs, 1000);
if(flag==1){

      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print(shoulderAngleDegrees);
      lcd.setCursor(6,1);
      lcd.print(shoulderAngleDegrees);
      lcd.setCursor(0,0);
      lcd.print(x);
      lcd.setCursor(6,0);
      lcd.print(z);
  shoulder.write(elbowAngleDegrees );    // shoulder
  elbo.write(elbowAngleDegrees  );    // elbow
  base.writeMicroseconds(b);    // elbow

  palm.writeMicroseconds(palm_ang);    // elbow
  finger.writeMicroseconds(pick_ang);    // elbow
}
  

delay(10);
}
