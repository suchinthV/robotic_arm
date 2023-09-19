#include <Servo.h>
#include <math.h>
// ramp lib
#include <Ramp.h> 

Servo servo1;
Servo servo2;
Servo servo3;

int servo1Offset = 2150;      // gripper closed - max 2200
int servo2Offset = 630;      // shoulder back - min 600  (1120 mid)
int servo3Offset = 2300;      // elbow back - max 2180 (1180 mid)

// moving the foot forwards or backwards in the side plane





double theta;
double alpha;
double shi;
float length=28;
float sholder1home=45;
float sholder2home=170;
float a1;
float a2;
double theta_degree,shi_degree;
float x,y;
float h;
int shoulderangle;
int elboangle;





//*****************************
class Interpolation {  
public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;    

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

Interpolation interpX;        // interpolation objects
Interpolation interpZ;
//*****************************

void setup() {

  servo1.attach(5);         // gripper
  servo2.attach(6);         // shoulder
  servo3.attach(10);         // elbow

  servo1.writeMicroseconds(servo1Offset);    // gripper
  servo2.writeMicroseconds(servo2Offset);    // shoulder
  servo3.writeMicroseconds(servo3Offset);    // elbow





  Serial.begin(9600);



}

void loop() {

         
     
      x=map(analogRead(A1),0,1024,1,56);
    
     
      y=map(analogRead(A0),0,1024,1,56);
      double y_range=sqrt((4*length*length)- (x*x));
      y=constrain(y,1,y_range);


      if((sq(x)+sq(y))<sq(2*length)){

          // step sequencer
          
//          z = interpZ.go(zTarget,100);
//          x = interpX.go(xTarget,100);

          // *** Inverse Kinematics ***
      
          // calculate modification to shoulder angle and arm length
      
          shi = atan(y/x);
          shi_degree = shi * (180.0/3.14);    // degrees
          h=sqrt((x*x)+(y*y)); 
          theta=acos(h/(2*length));
          theta_degree=  theta * (180/PI);
          a2=2*theta_degree;
          alpha=theta_degree+shi_degree;
          a1=180-alpha;
          shoulderangle=a1-45;
          elboangle=a2;



//          Serial.print(y_range);
//          Serial.print("   ");
//          Serial.print(shi);
//          Serial.print("   ");
//          Serial.print(a1);
//          Serial.print("   ");
//           Serial.print(h);
//          Serial.print("   ");
          
          Serial.print(x);
          Serial.print("   ");
          Serial.print(y);
          Serial.print("   ");
           Serial.print(shoulderangle);
          Serial.print("   ");
           Serial.print(elboangle);
          Serial.println("   ");
          
          

          

          if(! isnan(shoulderangle)) {
           servo2.write(shoulderangle);
          
          }
          if(! isnan(elboangle)) {
           servo3.write(elboangle);
          
          }
          delay(200);
         
      }    
}

         
      
