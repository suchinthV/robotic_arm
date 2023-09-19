
#include <Servo.h>
Servo servo1;
Servo servo2;
int zTarget, xTarget;
int x,z;

double shoulderAngle2;
double shoulderAngle2a;
double shoulderAngle2Degrees;
double shoulderAngle2aDegrees;
double z2;

double lowerLength = 28;
double upperLength = 28;
double armLength;


double shoulderAngle;
double shoulderAngle1;
double shoulderAngleDegrees;
double shoulderAngle1a;
double elbowAngle;
double elbowAngleDegrees;

int servo1home=620;
int servo2home=2300;
double shoulderMs;
double shoulderMs2;
double elbowMs;
bool flag=0,flag1=0;;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo1.attach(6);         // gripper
  servo2.attach(9);         // shoulder

   servo1.writeMicroseconds(servo1home);
   servo2.writeMicroseconds(servo2home);
   delay(4000);

}

void loop() {
  zTarget = map(analogRead(A0), 0, 1024, 500, 2500);
  xTarget = map(analogRead(A1), 0, 1024, 500, 2500);

//  Serial.print(zTarget);
//  Serial.print("     "  );
//  Serial.println(xTarget);
//  servo1.writeMicroseconds(zTarget);
//  servo2.writeMicroseconds(xTarget);

  delay(50);

  for (int i =1 ; i < 56; i+=10)
  { 
     x=i;
     flag=!flag;

    
   
    
    if(flag==1){
    for (int j = 1; j < 56; j+=5)
    {
      z=j;
     if((sq(x)+sq(z))>sq(56))
    {
     j=57;
    }
      
      
          shoulderAngle2a = atan(z/x);
          shoulderAngle2aDegrees = shoulderAngle2a * (180/PI);    // degrees
          shoulderAngle2 = shoulderAngle2a - 0.7853908;  // take away the default 45' offset (in radians)
          shoulderAngle2Degrees = shoulderAngle2 * (180/PI);    // degrees
          shoulderMs2 = shoulderAngle2Degrees * 9.5;
          
          z2 = x/cos(shoulderAngle2a);     // calc new arm length to feed to the next bit of code below
      
          // ****************
      
          // calculate arm length based on upper/lower length and elbow and shoulder angle
          shoulderAngle1a = (sq(upperLength) + sq(z2) - sq(lowerLength)) / (2 * upperLength * z2);
          shoulderAngle1 = acos(shoulderAngle1a);     // radians
          elbowAngle = PI - (shoulderAngle1 *2);       // radians
      
          // calc degrees from angles
          shoulderAngleDegrees = shoulderAngle1 * (180/PI);    // degrees
          elbowAngleDegrees = elbowAngle * (180/PI);              // degrees 
      
          // calc milliseconds PWM to drive the servo.
//          shoulderMs = shoulderAngleDegrees * 9.5;
//          elbowMs = elbowAngleDegrees * 9.5;

          // *** end of Inverse Kinematics ***
      
          // write to servos, remove 45' and 90' offsets from arm default position
//          int g=servo1home - (shoulderMs - 480) - shoulderMs2;
//          int h=servo2home + (elbowMs - 1000);

          int g=(170-shoulderAngleDegrees)-45;
          int h=170-elbowAngleDegrees;
            shoulderMs=(g*9.5)+630;
            elbowMs=(h*9.5)+630;

        if(!isnan(shoulderMs) && !isnan(elbowMs))
       {
       Serial.print(x);
       Serial.print("     "  );
       Serial.print(z);
       Serial.print("     "  );
       Serial.print(g);
       Serial.print("     "  );
       Serial.println(h);
       
       for(int k=servo1home;k<shoulderMs && flag1==0;k++)
       {
       servo1.writeMicroseconds(k);
       delay(20); 
       }
         for(int l=servo2home;l>elbowMs && flag1==0;l--)
       {
       servo2.writeMicroseconds(l);
       delay(20); 
       }
       flag1=1;
       servo1.write(g);
       
       servo2.write(h);
       
       }
      delay(200);
    }
    }
    if(flag==0)
    {
          for (int j = 56; j>1; j--)
    {
      z=j;
     while((sq(x)+sq(j))>sq(56))
    {
     j--;
    }
      
      

      
                shoulderAngle2a = atan(z/x);
          shoulderAngle2aDegrees = shoulderAngle2a * (180/PI);    // degrees
          shoulderAngle2 = shoulderAngle2a - 0.7853908;  // take away the default 45' offset (in radians)
          shoulderAngle2Degrees = shoulderAngle2 * (180/PI);    // degrees
          shoulderMs2 = shoulderAngle2Degrees * 11;
          
          z2 = x/cos(shoulderAngle2a);     // calc new arm length to feed to the next bit of code below
      
          // ****************
      
          // calculate arm length based on upper/lower length and elbow and shoulder angle
          shoulderAngle1a = (sq(upperLength) + sq(z2) - sq(lowerLength)) / (2 * upperLength * z2);
          shoulderAngle1 = acos(shoulderAngle1a);     // radians
          elbowAngle = PI - (shoulderAngle1 *2);       // radians
      
          // calc degrees from angles
          shoulderAngleDegrees = shoulderAngle1 * (180/PI);    // degrees
          elbowAngleDegrees = elbowAngle * (180/PI);              // degrees 
      
          // calc milliseconds PWM to drive the servo.
//          shoulderMs = shoulderAngleDegrees * 11;
//          elbowMs = elbowAngleDegrees * 11;

          // *** end of Inverse Kinematics ***
      
          // write to servos, remove 45' and 90' offsets from arm default position
//          int g=servo1home - (shoulderMs - 480) - shoulderMs2;
//          int h=servo2home + (elbowMs - 1000);

            int g=(170-shoulderAngleDegrees)-45;
          int h=170-elbowAngleDegrees;
           shoulderMs=(g*9.5)+630;
            elbowMs=(h*9.5)+630;


          
       if(!isnan(shoulderMs) && !isnan(elbowMs))
       {
       Serial.print(x);
       Serial.print("     "  );
       Serial.print(z);
       Serial.print("     "  );
       Serial.print(g);
       Serial.print("     "  );
       Serial.println(h);
       servo1.write(g);
       servo2.write(h);
       }
      delay(200);
    }
    }
    
  }



  }
