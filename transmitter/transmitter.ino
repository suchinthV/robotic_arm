// Basic demo for accelerometer readings from Adafruit MPU6050
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial myserial(7,8); //rx,tx
Adafruit_MPU6050 mpu;
Adafruit_MPU6050 mpu1;



#define MPU6050_I2C_ADDRESS 0x68   // Default MPU-6050 address
#define MPU6050_WHO_AM_I 0x75      // R Who am I address

void setup(void) {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
    WhoAmI();
  // Try to initialize!
   if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
    
  }
    if (!mpu1.begin(0x68)) {
    Serial.println("Failed to find MPU6050 2 chip");
    while (1) {
      delay(10);
    }
    
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);


  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
 myserial.begin(9600);

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print("  ");
  Serial.print(a.acceleration.y);
  Serial.print("   ");

  int n= map(a.acceleration.x,-10,10,-100,100);
  int o= map(a.acceleration.y,-10,10,-100,100);
  int pot_fin=analogRead(A0);
  int pot_z=analogRead(A2);
  int Gz=map(g.gyro.z,-10,10,-100,100);
  Serial.print(pot_fin);
  Serial.print("   ");
  Serial.print(pot_z);
  Serial.print("   ");
  Serial.println(Gz);
  // int p= map(a.acceleration.z,-10,10,-100,100);

    StaticJsonDocument<200> doc;
  doc["Ax"] = n;
  doc["Ay"] = o;
  doc["Ax2"] = n;
  doc["Ay2"] = o;
  doc["pot_fin"]=pot_fin;
  doc["pot_z"]=pot_z;
  doc["Gz"]=Gz;
  // doc["m"] = m;

   serializeJson(doc, myserial);
  

  Serial.println("");
  delay(200);
}
void WhoAmI(){
  uint8_t waiByte;                                    // Data will go here
  MPU6050_Read(MPU6050_WHO_AM_I, &waiByte);           // Get data
  Serial.print(F("Device WhoAmI reports as: 0x"));    // 
  Serial.println(waiByte,HEX);                        // Report WhoAmI data
  }

void MPU6050_Read(int address,uint8_t *data){            // Read from MPU6050. Needs register address, data array
  int size = sizeof(*data);                              //
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);           // Begin talking to MPU6050
  Wire.write(address);                                   // Set register address
  Wire.endTransmission(false);                           // Hold the I2C-bus
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);     // Request bytes, release I2C-bus after data read
  int i = 0;                                             //
  while(Wire.available()){                               //
    data[i++]=Wire.read();                               // Add data to array
  }
}