#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 bmp;
#define sampletime 1000
void setup()
{
  Serial.begin(9600);
}

void loop()
{
  get_bmp180();
  delay(sampletime);  // Pause for 5 seconds.
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
          Serial.println();
        }
        else Serial.print("BMP180_Tem::INVALID||BMP180_Alt::INVALID||BMP180_Pru::INVALID||BMP180_Prc::INVALID||\n");
      }
      else Serial.print("BMP180_Tem::INVALID||BMP180_Alt::INVALID||BMP180_Pru::INVALID||BMP180_Prc::INVALID||\n");
    }
    else Serial.print("BMP180_Tem::INVALID||BMP180_Alt::INVALID||BMP180_Pru::INVALID||BMP180_Prc::INVALID||\n");
  }
  else Serial.print("BMP180_Tem::INVALID||BMP180_Alt::INVALID||BMP180_Pru::INVALID||BMP180_Prc::INVALID||\n");

}
