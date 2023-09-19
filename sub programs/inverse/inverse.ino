#include <Servo.h>
// ramp lib
#include <Ramp.h>

Servo servo2;
Servo servo3;

int servo1Offset = 700;      // gripper closed - max 2200
int servo2Offset = 1120;      // shoulder back - min 600  (1120 mid)
int servo3Offset = 1180;      // elbow back - max 2180 (1180 mid)
int servo4Offset = 1555;
// moving the foot forwards or backwards in the side plane
double shoulderAngle2;
double shoulderAngle2a;
double shoulderAngle2Degrees;
double shoulderAngle2aDegrees;
double z2;
double x;

// side plane of individual leg only
double lowerLength = 280.0;
double upperLength = 180.0;
double armLength;
double z;
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

int ball;     // ball proximity

// wheel encoder interrupts

volatile long encoder0Pos = 0;    // encoder 1
long encoder0Target = 0;

unsigned long currentMillis;
unsigned long previousMillis;

long previousStepMillis = 0;    // set up timers
int stepFlag = 0;
int dl = 1000, u = 0, continew = 0, Stop = 0;

class Interpolation {  
public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue=10;    

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

void setup() {
  Serial.begin(9600);
    servo2.attach(6);         // shoulder
  servo3.attach(10);         // elbow
    servo2.writeMicroseconds(servo2Offset);    // shoulder
  servo3.writeMicroseconds(servo3Offset);    // elbow
  // put your setup code here, to run once:

}

void loop() {
 
//
  int p=analogRead(A0);
  p=map(p,0,1024,60,400);
  Serial.print("p :");
  Serial.print(p);
  Serial.print("    ");
   z= p;

  double y_range=sqrt(sq(460.0)-sq(z));
  Serial.print(y_range);
  Serial.print("    ");
  
  int k=analogRead(A1);
  k=map(k,0,1024,80,y_range);
  Serial.print("k  :");
  Serial.print(k);
   Serial.print("    ");
  x = k; 



 

  
  z2 = sqrt(sq(x) + sq(z));   // calc new arm length to feed to the next bit of code below
  //          Serial.print(" z2 =");
  Serial.print(z2);
  // ****************
  Serial.print("    ");

  shoulderAngle2a = (sq(x) + sq(z2) - sq(z)) / (2 * x * z2);
  double shi = acos(shoulderAngle2a);
 
  shoulderAngle1a = (sq(lowerLength) + sq(z2) - sq(upperLength)) / (2 * lowerLength * z2);


  double theeta = acos(shoulderAngle1a);     // radians
 


  shoulderAngle = theeta + shi;

  elbowAngle =  (sq(lowerLength)  + sq(upperLength) - sq(z2)) / (2 * lowerLength * upperLength );   // radians
  double alpha = acos(elbowAngle);

  elbowAngleDegrees = alpha * (180 / PI);
  Serial.print(elbowAngleDegrees);
  Serial.print("    ");
  shoulderAngleDegrees = shoulderAngle * (180 / PI);  // degrees
  Serial.print(shoulderAngleDegrees);

  shoulderMs = ((180-shoulderAngleDegrees) * 10.0 )+600;
  elbowMs = ((180-elbowAngleDegrees) * 9.0  )+600;


  float ang = interpX.go(shoulderMs,100);
  float ang2 = interpZ.go(elbowMs,100);
  if(! isnan(ang)){
  
  servo2.writeMicroseconds(ang );    // shoulder
   }
  if(! isnan(ang)){
  servo3.writeMicroseconds(ang2  );    // elbow  
  }

//  servo3.write(180-elbowAngleDegrees);    // shoulder
//  servo2.write(180- shoulderAngleDegrees);    // elbow  
 
//    servo2.write(90);    // shoulder
//  servo3.write(90 );    // elbow  

}
