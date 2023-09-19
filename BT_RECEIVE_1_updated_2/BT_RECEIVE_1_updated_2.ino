

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

int Ax, Ay, pot_fin=0, Gz, pot_z=0, pick_ang = 1500, palm_ang, b,Ax2,Ay2;



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
float shoulderms;
float elboms;

bool flag = 0, flag1 = 0;

//unsigned long currentMillis;//..................................................millis
//unsigned long previousMillis=millis();
unsigned long previousStepMillis = millis();    // set up timers




float pre_elboms, pre_shoulderms;




void setup() {

  base.attach(5);
  shoulder.attach(6);
  elbo.attach(10);

  palm.attach(3);
  finger.attach(11);
  Serial.begin(9600); // Default communication rate of the Bluetooth module


  shoulder.writeMicroseconds(630);
  base.writeMicroseconds(630);
  elbo.writeMicroseconds(2300);
  palm.writeMicroseconds(917);
  finger.writeMicroseconds(1500);
  pre_shoulderms = 630;
  pre_elboms = 2300;
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ROBOTIC ARM");
  lcd.setCursor(2, 1);
  lcd.print("working");
//    x = 10;
//    z = 10;
  b = 630;
  pick_ang=1500;

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

      Ax2 = doc["Ax2"].as<int>();
      Ay2 = doc["Ay2"].as<int>();

      pot_fin = doc["pot_fin"].as<int>();
      pot_z = doc["pot_z"].as<int>();
    

      if (millis() - previousStepMillis > 500)
      {
        previousStepMillis = millis();
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(Ax);
        lcd.print(",");
        lcd.print(Ay);
        lcd.print(",");
        lcd.print(pot_fin);
        lcd.print(",");
        lcd.print(pot_z);
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






  //    if (pot_fin > 600)
  //    {
  //      pick_ang += 50;
  //      pick_ang = constrain(pick_ang, 1500, 2300);
  //    }
  //    if (pot_fin < 600 && pot_fin > 350)
  //    {
  //      pick_ang -= 50;
  //      pick_ang = constrain(pick_ang, 1500, 2300);
  //    }
  //    if (pot_fin < 350)
  //    {
  //      pick_ang = pick_ang;
  //    }


  for (int j = x; Ax < -50; j++)//....................................x.............
  {
    x = j;
    x=constrain(x,10,50);
    if (x > 50)
    {
      break;
    }

    inversek();

    if (! isnan(elboangle) ) {
      elbo.writeMicroseconds(elboms);
    }

    if (! isnan(shoulderangle) && shoulderangle > 1 ) {
      shoulder.writeMicroseconds(shoulderms);
    }

    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        Ax = doc["Ax"].as<long>();
      }

    }
    delay(50);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("x loop forward");

  }

  for (int j = x; Ax > 50; j--)
  {
    x = j;
    x=constrain(x,10,56);
    if (x < 1)
    {
      break;
    }

    inversek();

    if (! isnan(elboangle)) {
      elbo.writeMicroseconds(elboms);
    }
    if (! isnan(shoulderangle) ) {
      shoulder.writeMicroseconds(shoulderms);
    }
    else
    {
      shoulderms-=100;
    }

    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        Ax = doc["Ax"].as<long>();
      }


    }
    delay(50);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("x loop back");
  }




  for (int k = z; Ax2 > 50 ; k++) //................................................Z....................
  {
    z = k;
    if (z > 50)
    {
      break;
    }

    inversek();

    if (! isnan(elboangle) &&  elboangle > 1) {
      elbo.writeMicroseconds(elboms);
    }
    if (! isnan(shoulderangle) && shoulderangle > 1 ) {
      shoulder.writeMicroseconds(shoulderms);
    }

    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        Ax2= doc["Ax2"].as<int>();
      }


    }
    delay(50);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("z loop up");
  }

  for (int k = z; Ax2 <-50 ; k--)
  {
    z = k;
    if (z < 1 )
    {
      break;
    }

    inversek();

    if (! isnan(elboangle) &&  elboangle > 1) {
      elbo.writeMicroseconds(elboms);
    }
    if (! isnan(shoulderangle) && shoulderangle > 1 ) {
      shoulder.writeMicroseconds(shoulderms);
    }

    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        Ax2 = doc["Ax2"].as<int>();
      }


    }
    delay(50);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("z loop down");
  }
  //....................................................y.........................................


  //    if (Ay < -50)
  //    {
  //      b += 100;
  //      b = constrain(b, 600, 2100);
  //
  //    }
  //    if (Ay > 50)
  //    {
  //      b -= 100;
  //      b = constrain(b, 600, 2100);
  //    }

  for (int j = b; Ay < -50; j+=10)
  {
    b = j;
    if (b > 2500 )
    {
      break;
    }


    if (! isnan(b) &&  b > 630) {
      base.writeMicroseconds(b);
    }


    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        Ay = doc["Ay"].as<long>();
      }


    }
    delay(1);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("y loop right");
  }

  for (int j = b; Ay > 50; j-=10)
  {
    b = j;
    if (b < 630 )
    {
      break;
    }



    if (! isnan(b) &&  b > 630) {
      base.writeMicroseconds(b);
    }


    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        Ay = doc["Ay"].as<long>();
      }


    }
    delay(1);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("y loop left");


  }
  //.....................................................................finger.............

  //    if (pot_fin > 600)
  //    {
  //      pick_ang += 50;
  //      pick_ang = constrain(pick_ang, 1500, 2300);
  //    }
  //    if (pot_fin < 600 && pot_fin > 350)
  //    {
  //      pick_ang -= 50;
  //      pick_ang = constrain(pick_ang, 1500, 2300);
  //    }
  //    if (pot_fin < 350)
  //    {
  //      pick_ang = pick_ang;
  //    }

  for (int j = pick_ang ; pot_fin > 600 ; j+=50)
  {
    pick_ang = j;
    if (pick_ang > 2300)
    {
      break;
    }


    if (! isnan(pick_ang) &&  pick_ang > 1500) {
      finger.writeMicroseconds(pick_ang);
    }


    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        pot_fin = doc["pot_fin"].as<long>();
      }


    }
    delay(50);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("finger loop cose");
  }


    for (int j = pick_ang ; pot_fin < 600 ; j-=50)
  {
    pick_ang = j;
    if (pick_ang < 1500 ||  pot_fin < 350)
    {
      break;
    }


    if (! isnan(pick_ang) &&  pick_ang > 1500) {
      finger.writeMicroseconds(pick_ang);
    }


    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        pot_fin = doc["pot_fin"].as<long>();
      }


    }
    delay(50);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("finger loop open");
  }

//.............................................................palm..................
  for (int j = palm_ang ; Ay2 > 50 ; j+=10)
  {
   palm_ang = j;
    if (palm_ang > 2100)
    {
      break;
    }


    if (! isnan(palm_ang)) {
      palm.writeMicroseconds(palm_ang);
    }


    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        Ay2= doc["Ay2"].as<long>();
      }


    }
    delay(5);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("palm loop cose");
  }


    for (int j = palm_ang ; Ay2< -50 ; j-=10)
  {
    palm_ang = j;
    if (palm_ang < 630)
    {
      break;
    }


    if (! isnan(palm_ang)) {
      palm.writeMicroseconds(palm_ang);
    }


    if (Serial.available() > 0) { // Checks whether data is comming from the serial port

      StaticJsonDocument<800> doc;

      DeserializationError err = deserializeJson(doc, Serial);

      if (err == DeserializationError::Ok)
      {

        Ay2 = doc["Ay2"].as<long>();
      }


    }
    delay(5);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("palm loop open");
  }

  

  

  delay(10);
}

void inversek()
{
  if ((sq(x) + sq(z)) < sq(2 * length)) {
    shi = atan(z / x);
    shi_degree = shi * (180.0 / 3.14);  // degrees
    h = sqrt((x * x) + (z * z));
    theta = acos(h / (2 * length));
    theta_degree =  theta * (180 / PI);
    a2 = 2 * theta_degree;
    alpha = theta_degree + shi_degree;
    a1 = 180 - alpha;
    shoulderangle = a1 - 30;
    elboangle = a2;
    shoulderms = ((shoulderangle * 9.0) + 600);
    elboms = ((elboangle * 9.0) + 600);


  }
  else
  {
   x--;
   z--;
  }
}
