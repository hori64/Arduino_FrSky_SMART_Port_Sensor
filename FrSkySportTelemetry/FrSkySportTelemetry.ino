#include <TinyGPS++.h>
#define GPSBaud  9600
TinyGPSPlus gps;

#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;
static double T = 0.0,
              P = 0.0,
              baseline;

#include "FrSkySportSensor.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorRpm.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#include "SoftwareSerial.h"
FrSkySportSensorGps gpss;      // Create GPS sensor with default ID
FrSkySportSensorRpm rpm;       // Create RPM sensor with default ID
FrSkySportSensorVario vario;   // Create Variometer sensor with default ID
FrSkySportTelemetry telemetry; // Create telemetry object

void setup()
{
  Serial.begin(GPSBaud); 
  pressure.begin();
  baseline = getPressure();
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_12, &gpss, &rpm, &vario);
}

void loop()
{
  double alt;
  P = getPressure();
  alt = pressure.altitude(P,baseline);
  if (gps.location.isValid())
  {
    gpss.setData(gps.location.lat(), gps.location.lng(),                 // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
                 gps.altitude.meters(),                                  // Altitude in m (can be negative)
                 gps.speed.kmph(),                                       // Speed
                 gps.course.deg(),                                       // Course over ground in degrees (0-359, 0 = north)
                 gps.date.year()-2000, gps.date.month(), gps.date.day(), // Date (year - 2000, month, day)
                 gps.time.hour(), gps.time.minute(), gps.time.second()); // Time (hour, minute, second) - will be affected by timezone setings in your radio
  };
    
  // Set RPM/temperature sensor data
  // (set number of blades to 2 in telemetry menu to get correct rpm value)
  rpm.setData(0,   // Rotations per minute
              T,   // Temperature #1 in degrees Celsuis (can be negative, will be rounded)
              T);  // Temperature #2 in degrees Celsuis (can be negative, will be rounded)

 // Set variometer data
  // (set Variometer source to VSpd in menu to use the vertical speed data from this sensor for variometer).
  vario.setData(alt,  // Altitude in meters (can be negative)
                -1.5);  // Vertical speed in m/s (positive - up, negative - down)
  smartDelay(100);
  if (digitalRead(5))digitalWrite(13,!digitalRead(13));
  else digitalWrite(13,HIGH);
}

double getPressure()
{
  char status;
  double p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
      }
    }
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    telemetry.send();
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}

