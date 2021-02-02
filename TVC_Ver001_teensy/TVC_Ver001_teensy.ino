#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Servo.h>

#include <SPI.h>
#include <Adafruit_BMP280.h>

#include <SoftwareSerial.h>


#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
 
Servo pitchServo;
Servo rollServo;

 
float q0;
float q1;
float q2;
float q3;
 
float rollTarget=0;
float rollActual;
float rollError;
float rollServoVal=88;
 
float pitchTarget=0;
float pitchActual;
float pitchError;
float pitchServoVal=95;

#define BNO055_SAMPLERATE_DELAY_MS (100)
 
Adafruit_BNO055 myIMU = Adafruit_BNO055();


void setup() {
  // put your setup code here, to run once:

Serial.begin(115200);
  //Serial.println(F("BMP280 test"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */



Serial.begin(115200);
myIMU.begin();
delay(1000);
int8_t temp=myIMU.getTemp();
myIMU.setExtCrystalUse(true);
rollServo.attach(2);
pitchServo.attach(3);

rollServo.write(100);
delay(20);
pitchServo.write(100);
delay(20);
rollServo.write(70);
delay(20);
pitchServo.write(70);
delay(20);
rollServo.write(rollServoVal);
delay(20);
pitchServo.write(pitchServoVal);
delay(20);

}
 
void loop() {
  // put your main code here, to run repeatedly:
  
uint8_t system, gyro, accel, mg = 0;
myIMU.getCalibration(&system, &gyro, &accel, &mg);
 
imu::Quaternion quat=myIMU.getQuat();
 
q0=quat.w();
q1=quat.x();
q2=quat.y();
q3=quat.z();
 
rollActual=-atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
pitchActual=-asin(2*(q0*q2-q3*q1));
 
rollActual=rollActual/(2*3.141592654)*360;
pitchActual=pitchActual/(2*3.141592654)*360;
 
rollError=rollTarget-rollActual;
pitchError=pitchTarget-pitchActual;

if (abs(pitchError)> 0.0) {
  pitchServoVal=pitchServoVal+pitchError/2;
  pitchServoVal = constrain(pitchServoVal,80,110); // X Axis
  pitchServo.write(pitchServoVal);
  delay(20);
}
 
 
if (abs(rollError)> 0.0) {
  rollServoVal=rollServoVal+rollError/2;
  rollServoVal = constrain(rollServoVal,70,100); //Y Axis
  rollServo.write(rollServoVal);
  delay(20);
}


Serial.print(rollServoVal);
Serial.print(",");
Serial.print(rollTarget);
Serial.print(",");
Serial.print(rollActual);
Serial.print(",");
Serial.print(pitchTarget);
Serial.print(",");
Serial.print(pitchActual);
Serial.print(",");
Serial.print(accel);
Serial.print(",");
Serial.print(gyro);
Serial.print(",");
Serial.print(mg);
Serial.print(",");
Serial.print(system);
Serial.print(",");
Serial.print(bmp.readTemperature());
Serial.print(",");
Serial.print(bmp.readPressure());
Serial.print(",");
//Adjusted readAltitude to local forecast!
Serial.println(bmp.readAltitude(1014));

delay(BNO055_SAMPLERATE_DELAY_MS);
}
