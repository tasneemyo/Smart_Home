#include "LiquidCrystal_I2C.h"
#include "DHT.h"
#include <Servo.h>
// C++ code
//
//int tempSensor=0;

#define DHTPIN 3      
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

int trig=7;
int echo=6;
int duration = 0;
int distance = 0;
int pos=0;
int buzzer1=2;
int buzzer2=4;
byte flameAnalog=A0;
int flameDigital=9;
LiquidCrystal_I2C lcd_1(0x27,16,2);
byte DC=A2;
void setup()
{
  dht.begin(); 
  pinMode(trig , OUTPUT);
  pinMode(echo , INPUT);
  pinMode(buzzer1 , OUTPUT);
  pinMode(buzzer2 , OUTPUT);
  pinMode(flameAnalog,INPUT);
  pinMode(flameDigital,INPUT);
  pinMode(DC,OUTPUT);
  Serial.begin(9600); 
  lcd_1.init();
  lcd_1.backlight();
}

void loop()
{
 
 digitalWrite(trig, LOW);
 delayMicroseconds(2);
 digitalWrite(trig, HIGH);
 delayMicroseconds(10);
 digitalWrite(trig, LOW);
 duration = pulseIn(echo, HIGH);
 distance= (duration*0.034/2);
//  digitalWrite(buzzer1, LOW);
 if (distance<15)
    digitalWrite(buzzer1, HIGH);
  else 
    digitalWrite(buzzer1, LOW);
 

  float h = dht.readHumidity();
  delay(10);
  float t = dht.readTemperature();
lcd_1.setCursor(0, 0);
 lcd_1.setBacklight(1);
 lcd_1.print("Temprature =");
 lcd_1.print(t);

 // delay(1000);
 lcd_1.setCursor(0, 1);
lcd_1.print("Humidity = ");
  lcd_1.print(h);
  int readingFlame=0;
  readingFlame=analogRead(flameAnalog);
    Serial.println(readingFlame);
    // digitalWrite(buzzer2,LOW);
  if (readingFlame<1020)
      digitalWrite(buzzer2,HIGH);
  else 
      digitalWrite(buzzer2,LOW);

  if (t>30)
    analogWrite(DC,100);
  else 
    analogWrite(DC,0);
  delay(500);
  

}
