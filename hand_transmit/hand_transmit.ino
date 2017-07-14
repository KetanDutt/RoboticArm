//#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <VirtualWire.h>

uint8_t msg[7];

MPU6050 mpu;

int pos = 0; 

const int flexpin1 = 0;
const int flexpin2 = 1;
const int flexpin3 = 2;
const int flexpin4 = 3;
const int flexpin5 = 4;

int flexposition1;
int flexposition2;
int flexposition3;
int flexposition4;
int flexposition5;

int servoposition1;
int servoposition2;
int servoposition3;

int servoposition4;
int servoposition5;

int16_t ax, ay, az;
int16_t gx, gy, gz;

/*Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
*/
int val1;
int val2;
int val3;
int val4;
int val5;
int val6;

int prevVal1;
int prevVal2;
int prevVal3;
int prevVal4;
int prevVal5;
int prevVal6;

void setup() 
{   Wire.begin();
    Serial.begin(38400);
    
    vw_set_ptt_inverted(true);
    vw_setup(2000);

    mpu.initialize();
/*    
    myservo1.attach(3);
    myservo2.attach(9);
    myservo3.attach(4);
    myservo4.attach(5);
    myservo5.attach(6);
    myservo6.attach(7);
    myservo7.attach(8);*/
}

void loop() 
{   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    val1 = map(ax, 17000, -17000, 0, 179);
    val2 = map(ay, -17000, 17000, 0, 179);
    val3 = map(az, -17000, 17000, 0, 179);
    val4 = map(gz, -17000, 17000, 0, 179);
    val5 = map(gz, -17000, 17000, 0, 179);
    val6 = map(gz, -17000, 17000, 0, 179);
    
    Serial.println("Gyro:- ");
    Serial.print(ax);
/*    Serial.print(":");
    Serial.print(ay);
    Serial.print(":");
    Serial.print(az);
    Serial.print("/t");
    Serial.print(gx);
    Serial.print(":");
    Serial.print(gy);
    Serial.print(":");
    Serial.print(gz);
   */ 
    if (val1 != prevVal1)
    {   msg[0]=val1;// myservo1.write(val1);
        prevVal1 = val1;
    }
    
    if (val2 != prevVal2)
    {   msg[1]=val2;//myservo2.write(val2);
        prevVal2 = val2;
    }

    flexposition1 = analogRead(flexpin1);
    flexposition2 = analogRead(flexpin2);
    flexposition3 = analogRead(flexpin3);
    flexposition4 = analogRead(flexpin4);
    flexposition5 = analogRead(flexpin5);

    servoposition1 = map(flexposition1, 220, 90, 0, 180);
    servoposition2 = map(flexposition2, 220,90, 0, 180);
    servoposition3 = map(flexposition3, 90, 220, 0, 180);
    servoposition4 = map(flexposition4, 220, 90, 0, 180);
    servoposition5 = map(flexposition5, 90, 220, 0, 180);

    servoposition1 = constrain(servoposition1, 0, 180);
    servoposition2 = constrain(servoposition2, 0, 180);
    servoposition3 = constrain(servoposition3, 0, 180);
    servoposition4 = constrain(servoposition4, 0, 180);
    servoposition5 = constrain(servoposition5, 0, 180);
/*
    myservo3.write(servoposition1);
    myservo4.write(servoposition2);
    myservo5.write(servoposition3);
    myservo6.write(servoposition4);
    myservo7.write(servoposition5);
*/
    msg[2] = servoposition1;
    msg[3] = servoposition2;
    msg[4] = servoposition3;
    msg[5] = servoposition4;
    msg[6] = servoposition5;
    
    vw_send(msg, 7); 
    vw_wait_tx(); 
    delay(10);
}
