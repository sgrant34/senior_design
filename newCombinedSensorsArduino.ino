#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <JY901.h>
#include "TimerOne.h"          // Timer Interrupt set to 2 second for read sensors
#include <math.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 2

#define WindSensorPin (3)      // The digital pin location of the anemometer sensor
#define WindVanePin (A3)       // The analog pin the wind vane sensor is connected to
#define VaneOffset 0;        // define the anemometer offset from magnetic north

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);	

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

Adafruit_BME280 bme; // I2C (SDA/SCL)
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;
int VaneValue;       // raw analog value from wind vane
int Direction;       // translated 0 - 360 direction
int CalDirection;    // converted value with offset applied
int LastValue;

volatile bool IsSampleRequired;       // this is set true every 2.5s. Get wind speed
volatile unsigned int  TimerCount;    // used to determine 2.5sec timer count
volatile unsigned long Rotations;     // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime;  // Timer to avoid contact bounce in interrupt routine

float WindSpeed;        // speed miles per hour

void setup() {
  LastValue = 0;
  IsSampleRequired = false;
  TimerCount = 0;
  Rotations = 0;   // Set Rotations to 0 ready for calculations
  sensors.begin();	// Start up the library
  Serial.begin(9600);
  Serial.println(F("M81 Sensors test"));
  JY901.StartIIC();
  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);

  // Setup the timer intterupt
  Timer1.initialize(500000);
  Timer1.attachInterrupt(isr_timer);

  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.println("-- Default Test --");
  delayTime = 1000;
}

void loop() { 
  // tri-sensor data
  delay(delayTime);
    // Send the command to get temperatures
  sensors.requestTemperatures(); 
  //water temperature sensor data
  //print the temperature in Celsius
  int temporary,runTime = 0;
  float waterTemp,totalWaterTemp,triTemp,totalTriTemp,triPressure,totalTriPressure, triHumidity, totalTriHumidity, windSpeed, totalWindSpeed, windDirection, totalWindDirection = 0;
  temporary = 0; totalWaterTemp = 0; totalTriTemp = 0; totalTriPressure = 0; totalTriHumidity = 0; totalWindDirection = 0; totalWindSpeed = 0; runTime = 0;

  while (temporary < 60){                         //program should run about 60 seconds every 10 minutes 
    sensors.requestTemperatures(); 
    getWindDirection();

    waterTemp = (sensors.getTempCByIndex(0));     //collects water temp in a temporary variable
    totalWaterTemp = totalWaterTemp + waterTemp;

    triTemp = (bme.readTemperature());            //collects air temp in a temporary variable
    totalTriTemp = totalTriTemp + triTemp;

    triPressure = (bme.readPressure() / 100.0F);  //collects pressure in a temporary variable
    totalTriPressure = totalTriPressure + triPressure;

    triHumidity = (bme.readHumidity());           //collects humidity in a temporary variable
    totalTriHumidity = totalTriHumidity + triHumidity;

    windSpeed = (Rotations * 0.9);                //collects wind speed in a temporary variable
     Rotations = 0;
    totalWindSpeed = totalWindSpeed + windSpeed;
    
    windDirection = (CalDirection);               //collects wind direction in a temporary variable
    totalWindDirection = totalWindDirection + windDirection;
    temporary = temporary + 1;
    delay(1000);
  }
  runTime = runTime + 1;  
  totalWaterTemp = totalWaterTemp/60;           //averages all values measured in for loop
  totalTriTemp = totalTriTemp/60;
  totalTriPressure = totalTriPressure/60;
  totalTriHumidity = totalTriHumidity/60;
  totalWindDirection = totalWindDirection/60;
  totalWindSpeed = totalWindSpeed/60;

  Serial.print("Run Time = ");
  Serial.print(runTime);
  Serial.println(" mins");
  Serial.println("Water Temperature: ");
  Serial.print(totalWaterTemp);
  //Serial.print(sensors.getTempCByIndex(0));
  Serial.print((char)176);//shows degrees character
  Serial.print("C  |  ");
  
  //print the temperature in Fahrenheit
  Serial.print((totalWaterTemp * 9.0) / 5.0 + 32.0);
  //Serial.print((sensors.getTempCByIndex(0) * 9.0) / 5.0 + 32.0);
  Serial.print((char)176);//shows degrees character
  Serial.println("F");
  
  Serial.print("Trisensor Temperature = ");
  Serial.print(totalTriTemp);
  Serial.println(" *C");

  Serial.print("Trisensor Pressure = ");
  Serial.print(totalTriPressure);
  Serial.println(" hPa");

  Serial.print("Trisensor Humidity = ");
  Serial.print(totalTriHumidity);
  Serial.println(" %");
  
  //Anemometer Data
  getWindDirection();

  // Only update the display if change greater than 5 degrees. 
  if(abs(CalDirection - LastValue) > 5)
  { 
     LastValue = CalDirection;
  }

  if(IsSampleRequired)
  {
     // convert to mp/h using the formula V=P(2.25/T)
     // V = P(2.25/2.5) = P * 0.9
     WindSpeed = Rotations * 0.9;
     Rotations = 0;   // Reset count for next sample
     
     IsSampleRequired = false; 
     
     Serial.print("Wind Speed(MPH): ");Serial.println(totalWindSpeed);    
     //Serial.print("Wind Speed(Knots): ");Serial.println(getKnots(WindSpeed));    
     Serial.print("Wind Direction: ");Serial.print(totalWindDirection); 
     getHeading(CalDirection);   Serial.println("\t\t"); 
     Serial.print("Calculated Wind Strength: ");getWindStrength(totalWindSpeed);
  }
  totalWaterTemp = 0; totalTriTemp = 0; totalTriPressure = 0; totalTriHumidity = 0; totalWindDirection = 0; totalWindSpeed = 0; temporary = 0;
  delay(540000);
  runTime = runTime + 9;
}
// isr routine fr timer interrupt
void isr_timer() {
  
  TimerCount++;
  
  if(TimerCount == 5)
  {
    IsSampleRequired = true;
    TimerCount = 0;
  }
}

// This is the function that the interrupt calls to increment the rotation count
void isr_rotation ()   {

  if ((millis() - ContactBounceTime) > 15 ) {  // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }

}
// Convert MPH to Knots
float getKnots(float speed) {
   return speed * 0.868976;
}
// Get Wind Direction
void getWindDirection() {
 
   VaneValue = analogRead(WindVanePin);
   Direction = map(VaneValue, 0, 1023, 0, 360);
   CalDirection = Direction + VaneOffset;
   
   if(CalDirection > 360)
     CalDirection = CalDirection - 360;
     
   if(CalDirection < 0)
     CalDirection = CalDirection + 360;
  
}
// Converts compass direction to heading
void getHeading(int direction) {
    
    if(direction < 22)
      Serial.print(" N");
    else if (direction < 67)
      Serial.print(" NE");
    else if (direction < 112)
      Serial.print(" E");
    else if (direction < 157)
      Serial.print(" SE");
    else if (direction < 212)
      Serial.print(" S");
    else if (direction < 247)
      Serial.print(" SW");
    else if (direction < 292)
      Serial.print(" W");
    else if (direction < 337)
      Serial.print(" NW");
    else
      Serial.print(" N");  
}
// converts wind speed to wind strength
void getWindStrength(float speed)
{
   
  if(speed < 2)
    Serial.println("Calm");
  else if(speed >= 2 && speed < 4)
    Serial.println("Light Air");
  else if(speed >= 4 && speed < 8)
    Serial.println("Light Breeze");
  else if(speed >= 8 && speed < 13)
    Serial.println("Gentle Breeze");
  else if(speed >= 13 && speed < 18)
    Serial.println("Moderate Breeze");
  else if(speed >= 18 && speed < 25)
    Serial.println("Fresh Breeze");
  else if(speed >= 25 && speed < 31)
    Serial.println("Strong Breeze");
  else if(speed >= 31 && speed < 39)
    Serial.println("Near Gale");
  else
    Serial.println("RUN");
}