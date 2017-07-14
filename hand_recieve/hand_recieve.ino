//#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <VirtualWire.h>
#include <ServoTimer2.h> 

uint8_t msg[2];
uint8_t buflen;

/*MPU6050 mpu;

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
*/
ServoTimer2 myservo1;
ServoTimer2 myservo2;
ServoTimer2 myservo3;
ServoTimer2 myservo4;
ServoTimer2 myservo5;
ServoTimer2 myservo6;
ServoTimer2 myservo7;

/*int val1;
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
*/
void setup() 
{   Wire.begin();
    Serial.begin(38400);
   // mpu.initialize();
    
    buflen = 7;
    vw_setup(2000);
    vw_rx_start(); 
    
    myservo1.attach(3);
    myservo2.attach(9);
    myservo3.attach(4);
    myservo4.attach(5);
    myservo5.attach(6);
    myservo6.attach(7);
    myservo7.attach(8);
}

void loop() 
{  if (vw_get_message(msg, &buflen)) 
    {     myservo1.write(msg[0]);
          Serial.print(  msg[0]  );
          myservo2.write(msg[1]);
          Serial.print(  msg[1]  );
          myservo2.write(msg[2]);
          Serial.print(  msg[2]  );
          myservo2.write(msg[3]);
          Serial.print(  msg[3]  );
          myservo2.write(msg[4]);
          Serial.print(  msg[4]  );
          myservo2.write(msg[5]);
          Serial.print(  msg[5]  );
          myservo2.write(msg[6]);
          Serial.print(  msg[6]  );
    }
  
  
  /* mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    val1 = map(ax, 17000, -17000, 0, 179);
    val2 = map(ay, -17000, 17000, 0, 179);
    val3 = map(az, -17000, 17000, 0, 179);
    val4 = map(gz, -17000, 17000, 0, 179);
    val5 = map(gz, -17000, 17000, 0, 179);
    val6 = map(gz, -17000, 17000, 0, 179);
    
    Serial.println("Gyro:- ");
    Serial.print(ax);
    Serial.print(":");
    Serial.print(ay);
    Serial.print(":");
    Serial.print(az);
    Serial.print("/t");
    Serial.print(gx);
    Serial.print(":");
    Serial.print(gy);
    Serial.print(":");
    Serial.print(gz);
    
    if (val1 != prevVal1)
    {    myservo1.write(val1);
        prevVal1 = val1;
    }
    
    if (val2 != prevVal2)
    {   myservo2.write(val2);
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

    myservo3.write(servoposition1);
    myservo4.write(servoposition2);
    myservo5.write(servoposition3);
    myservo6.write(servoposition4);
    myservo7.write(servoposition5);

   /* Serial.println("Flex:- ");
    Serial.print(flexposition1);
    Serial.print(":");
    Serial.print(flexposition2);
    Serial.print(":");
    Serial.print(flexposition3);
    Serial.print(":");
    Serial.print(flexposition4);
    Serial.print(":");
    Serial.print(flexposition5);
    
    delay(20); 
    
/*    for(pos = 60; pos <= 120; pos += 1) 
  {                               
    myservo1.write(pos);           
    delay(50);           
  } 
  
  for(pos = 45; pos <= 135; pos += 1) 
  {                                  
    myservo2.write(pos);            
    delay(20);                   
  } 
  
     for(pos = 0; pos <= 180; pos += 1)  
  {                                 
    myservo3.write(pos);            
    delay(5);                   
  } 
 
   for(pos = 0; pos <= 180; pos += 1)
  {                                  
    myservo4.write(pos);             
    delay(5);                     
  } 
  
  for(pos = 120; pos>=60; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo1.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(50);                       // waits 15ms for the servo to reach the position 
  } 
  
  for(pos = 135; pos>=45; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo2.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(20);                       // waits 15ms for the servo to reach the position 
  } 
  
   for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo3.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(5);                       // waits 15ms for the servo to reach the position 
  } 
  
   for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo4.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(5);                       // waits 15ms for the servo to reach the position 
  } 
    delay(50);*/
}
