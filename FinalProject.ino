#include "LiquidCrystal_I2C.h"
#include "DHT.h"
#include <Servo.h>
// C++ code
//
//int tempSensor=0;

#define DHTPIN 3      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

int trig=7;
int echo=6;
int duration = 0;
int distance = 0;
// int buzzer1=5;
int DC=A1;
byte ldr = A3;

int ldrReading = 0;
int minIntensity = 50; //test and change
// int PIR=2;
// int led=3;
int pos=0;
int buzzer1=2;

//Adafruit_LiquidCrystal lcd_1(0);
LiquidCrystal_I2C lcd_1(0x27,16,2);
Servo servo1;
void setup()
{
  dht.begin(); 
  pinMode(trig , OUTPUT);
  pinMode(echo , INPUT);
  pinMode(buzzer1 , OUTPUT);
  // pinMode(buzzer1 , OUTPUT);
  // pinMode(DC,OUTPUT);
  pinMode(ldr, INPUT);
//  pinMode(tempSensor, INPUT);
  // pinMode(PIR,INPUT);
  // pinMode(led,OUTPUT);
  servo1.attach(9);
  for(int i=0; i<=90;i++){
    servo1.write(i);
  }
  Serial.begin(9600); 
  lcd_1.init();
  lcd_1.backlight();
}

void loop()
{
  //--------distance--------
 //lcd_1.begin(16, 2);
 //lcd_1.clear();
 digitalWrite(trig, LOW);
 delayMicroseconds(2);
 digitalWrite(trig, HIGH);
 delayMicroseconds(10);
 digitalWrite(trig, LOW);
 duration = pulseIn(echo, HIGH);
 distance= (duration*0.034/2);
 if (distance<15)
    digitalWrite(buzzer1, HIGH);
  else 
    digitalWrite(buzzer1, LOW);
 
// int reading = analogRead(tempSensor);
  // float voltage = reading * 5.0;
  // voltage/=1024.0; 
   //float C = (voltage-0.5)*100;
    float h = dht.readHumidity();
  delay(10);
  float t = dht.readTemperature();
lcd_1.setCursor(0, 0);
 lcd_1.setBacklight(1);
 lcd_1.print("Temprature =");
 lcd_1.print(t);
Serial.println(distance);
  delay(1000);
 lcd_1.setCursor(0, 1);
lcd_1.print("Humidity = ");
  lcd_1.print(h);


 //lcd_1.print(voltage);

 //-----------temprature----------- 
    
  // lcd_1.setCursor(0, 0);
  // lcd_1.print(C);
  // lcd_1.print("degrees");
  // lcd_1.setCursor(0, 1);
  // if (C>20)
  //   digitalWrite(DC,HIGH);
  // else 
  //   digitalWrite(DC,LOW);
  // //---------PIR----------
	// digitalWrite(led,digitalRead(PIR));
  // delay(10);
  // lcd_1.setBacklight(1);
  // delay(500); // Wait for 500 millisecond(s)
  // // Wait for 500 millisecond(s)
 
//Serial.println(C);
  delay(1000);
  
  //--------shutters--------
  ldrReading = analogRead(ldr);
  if(ldrReading<= minIntensity){
    //clockwise (check)
    Serial.println(ldrReading);
    for(pos = 90; pos<=180; pos++){
      servo1.write(pos);
    }
  }
  else
  {
    for(pos = 180; pos<=90 ; pos--){
      servo1.write(pos);
    }
  }
}
