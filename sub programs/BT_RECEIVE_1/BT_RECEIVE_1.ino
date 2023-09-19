

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <Ramp.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <ArduinoJson.h>

Servo base;
Servo elbo;
Servo shoulder;
Servo palm;
Servo finger;

int k, l, m;
int motion;


int baseAngle = 1400, palmAngle = 1400, fingerAngle = 1400;

int Ax, Ay, Az, pot_fin, Gz, pot_z, pick_ang = 1500, palm_ang, b;



int servo1Offset = 2150;      // gripper closed - max 2200
int servo2Offset = 630;      // shoulder back - min 600  (1120 mid)
int servo3Offset = 2300;      // elbow back - max 2180 (1180 mid)
int servo4Offset = 1555;

double zTarget;
double xTarget;


double theta;
double alpha;
double shi;
float length = 28; //...................................in cm
float sholder1home = 45;
float elbohome = 170;
float a1;
float a2;
double theta_degree, shi_degree;
float x, z;
float h;
int shoulderangle;
int elboangle;
int shoulderms;
int elboms;

bool flag = 0;

//unsigned long currentMillis;//..................................................millis
//unsigned long previousMillis=millis();
unsigned long previousStepMillis = millis();    // set up timers


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
float pre_elboms,pre_shoulderms;




void setup() {

  base.attach(5);
  shoulder.attach(6);
  elbo.attach(10);

  palm.attach(11);
  finger.attach(3);
  Serial.begin(9600); // Default communication rate of the Bluetooth module


  shoulder.writeMicroseconds(630);
  base.writeMicroseconds(630);
  elbo.writeMicroseconds(2300);
  palm.writeMicroseconds(917);
  finger.writeMicroseconds(1500);
   pre_shoulderms=630;
   pre_elboms=2300;
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ROBOTIC ARM");
  lcd.setCursor(2, 1);
  lcd.print("working");
  xTarget=10;
  zTarget=10;

}

void loop() {
  if (Serial.available() > 0) { // Checks whether data is comming from the serial port

    StaticJsonDocument<800> doc;

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

      if (millis() - previousStepMillis > 500)
      {
        previousStepMillis = millis();
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(Ax);
        lcd.setCursor(4, 1);
        lcd.print(Ay);
        lcd.setCursor(8, 1);
        lcd.print(pot_fin);
        //        lcd.setCursor(0, 0);
        //        lcd.print(z);
        //        lcd.setCursor(6, 0);
        //        lcd.print(x);

        flag = 1;
      }


    }
    else
    {
      //        Print error to the "debug" serial port
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("deserializeJson() returned ");
      lcd.setCursor(0, 1);
      lcd.print(err.c_str());
      flag = 0;
      // Flush all bytes in the "link" serial port buffer

    }


  }
  else
  {
    flag = 0;
  }




if(flag==1){
  
  if (Ax < -50)
  {
    xTarget+=10;
   xTarget = constrain(xTarget, 1, 2 * length);

  }
  if (Ax > 50)
  {
   xTarget-=10;
   xTarget= constrain(xTarget, 1, 2 * length);
  }
  if (Ay < -50)
  {
    b += 100;
    b = constrain(b, 600, 2100);

  }
  if (Ay > 50)
  {
    b -= 100;
    b = constrain(b, 600, 2100);
  }
  ///////////////
  double z_range = sqrt((4 * length * length) - (x * x));

  if (pot_z < 250)
  {
    zTarget-=10;
    zTarget = constrain(zTarget, 1, z_range);
  }
  if (pot_z > 250 && pot_z < 450)
  {
    zTarget+=10;
    zTarget = constrain(zTarget, 1, z_range);
  }
  if (pot_z > 450)
  {
    zTarget = z;
  }


  ////////////////
  if (pot_fin > 600)
  {
    pick_ang += 50;
    pick_ang = constrain(pick_ang, 1500, 2300);
  }
  if (pot_fin < 600 && pot_fin > 350)
  {
    pick_ang -= 50;
    pick_ang = constrain(pick_ang, 1500, 2300);
  }
  if (pot_fin < 350)
  {
    pick_ang = pick_ang;
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




  if ((sq(x) + sq(z)) < sq(2 * length)) {

    // step sequencer

         z = interpZ.go(zTarget,100);
         x = interpX.go(xTarget,100);

    // *** Inverse Kinematics ***

    // calculate modification to shoulder angle and arm length

    shi = atan(z / x);
    shi_degree = shi * (180.0 / 3.14);  // degrees
    h = sqrt((x * x) + (z * z));
    theta = acos(h / (2 * length));
    theta_degree =  theta * (180 / PI);
    a2 = 2 * theta_degree;
    alpha = theta_degree + shi_degree;
    a1 = 180 - alpha;
    shoulderangle = a1 - 45;
    elboangle = a2;

    shoulderms = (shoulderangle * 9.5) + 630;
    elboms = (shoulderangle * 9.5) + 630;

    //          Serial.print(y_range);
    //          Serial.print("   ");
    //          Serial.print(shi);
    //          Serial.print("   ");
    //          Serial.print(a1);
    //          Serial.print("   ");
    //           Serial.print(h);
    //          Serial.print("   ");

    //      Serial.print(x);
    //      Serial.print("   ");
    //      Serial.print(z);
    //      Serial.print("   ");
    //      Serial.print(shoulderangle);
    //      Serial.print("   ");
    //      Serial.print(elboangle);
    //      Serial.println("   ");





    
    if (! isnan(elboms)  ) {
      elbo.writeMicroseconds(elboms);
      pre_elboms=elboms;
    }
    if (! isnan(shoulderms) ) {
      shoulder.writeMicroseconds(shoulderms);
      pre_shoulderms=shoulderms;
    }
  }

  base.writeMicroseconds(b);    // elbow

  palm.writeMicroseconds(palm_ang);    // elbow
  finger.writeMicroseconds(pick_ang);    // elbow
  
}
delay(10);
}
