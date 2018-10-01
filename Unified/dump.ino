
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SFE_BMP180.h>

//--------Constants----------
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
#define DHTPIN            5
#define DHTTYPE           DHT11
uint32_t delayMS;   

//--------Initializing objects---------
RTC_DS1307 rtc;
TinyGPSPlus gps;
DHT_Unified dht(DHTPIN, DHTTYPE);
SFE_BMP180 bmp;

SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  rtc_check();
  gps_check();
  dht_check();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  displayInfo();
     //get_rtc();
  smartDelay(1000);    
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
 
  
}

void displayInfo()  {
  DateTime now = rtc.now();
   
  //----------GPS Date----------
  Serial.print(F("GPS_Date::"));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.year());
    Serial.print(F("/"));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("||"));
  }
  else
  {
    error();
  }
  //----------GPS Time----------
  Serial.print(F("GPS_Time::"));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("||"));
  }
  else  {
    error();
  }
  //----------RTC Date----------
    Serial.print("RTC_Date::");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print("||");
  //----------RTC Time----------
    Serial.print("RTC_Time::");
    if (now.hour() < 10) Serial.print(F("0"));
    Serial.print(now.hour(),DEC);
    Serial.print(F(":"));
    if (now.minute() < 10) Serial.print(F("0"));
    Serial.print(now.minute(),DEC);
    Serial.print(F(":"));
    if (now.second() < 10) Serial.print(F("0"));
    Serial.print(now.second(),DEC);
    Serial.print(F("||"));
  //----------GPS Coordinate----------
  Serial.print("GPS_Lat::");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F("||"));
  }
  else {
    error();
  }
  Serial.print("GPS_Lng::");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lng(), 6);
    Serial.print(F("||"));
  }
  else
  {
    error();
  }
  //----------GPS Altitude and speed----------
  Serial.print("GPS_Spd::");
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.mps());
    Serial.print(F("||"));
  }
  else
    error();
    
  Serial.print("GPS_Alt::");
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters(), 2);
    Serial.print(F("||"));
  }
  else
    error();
  
 //----------DHT Measurements-----------------
  dht_print();

 //----------BMP180 Measurements--------------
  get_bmp180();
  Serial.println();
 
}

void error()  {
  Serial.print(F("INVALID"));
  Serial.print(F("||"));
  
}

void rtc_check() {
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

void gps_check()  {
  ss.begin(GPSBaud);
}

void dht_check() {
  dht.begin();
  sensor_t sensor;
  delayMS = sensor.min_delay / 1000;
}

void dht_print() {
  //delay(delayMS);
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) error();
  else {
    Serial.print("DHT11_Temp::");
    Serial.print(event.temperature);
    Serial.print(F("||"));
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) error();
  else {
    Serial.print("DHT11_Hum::");
    if (event.relative_humidity < 10) Serial.print(F("0"));
    Serial.print(event.relative_humidity);
    Serial.print(F("||"));
  }

}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void bmp_init() {
  if (bmp.begin())
    Serial.print("BMP180_Sta::ON-LINE||");
  else
  {
    Serial.print("BMP180_Sta::OFFLINE||");
  }
}

void get_bmp180() {
  bmp_init();
  char status;
  double T,P,p0,a;

  status = bmp.startTemperature();
  
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = bmp.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("BMP180_Tem::");
      Serial.print(T,4);
      Serial.print("||"); 
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = bmp.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = bmp.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb
          p0 = bmp.sealevel(P,36.576);    // Thiruninravur (altitude from sealevel in metres)
          Serial.print("BMP180_Alt::");
          Serial.print(36.576,4);
          Serial.print("||");
          Serial.print("BMP180_Pru::");
          Serial.print(P,2);
          Serial.print("||");
          Serial.print("BMP180_Prc::");
          Serial.print(p0,2);
          Serial.print("||");
      //    Serial.println();
        }
        else Serial.print("BMP180_Tem::INVALID||BMP180_Alt::INVALID||BMP180_Pru::INVALID||BMP180_Prc::INVALID||\n");
      }
      else Serial.print("BMP180_Tem::INVALID||BMP180_Alt::INVALID||BMP180_Pru::INVALID||BMP180_Prc::INVALID||\n");
    }
    else Serial.print("BMP180_Tem::INVALID||BMP180_Alt::INVALID||BMP180_Pru::INVALID||BMP180_Prc::INVALID||\n");
  }
  else Serial.print("BMP180_Tem::INVALID||BMP180_Alt::INVALID||BMP180_Pru::INVALID||BMP180_Prc::INVALID||\n");
  
}
