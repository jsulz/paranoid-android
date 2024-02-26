/******************************************************************************
  The code below was aided greatly by:
    SparkFun Photon Weather Shield basic example
    Authors: Joel Bartlett @ SparkFun Electronics
    Original Creation Date: May 18, 2015
    URL: https://github.com/sparkfun/Photon_Weather_Shield/blob/master/Firmware/SparkFun_Photon_Weather_Basic_Soil/SparkFun_Photon_Weather_Basic_Soil.ino

  And also by:
    weatherstation
    Author:
    Original creation Date:
    URL: https://github.com/rpurser47/weatherstation/tree/master
*******************************************************************************/

/**************************
Helping libraries
*************************/
#include "application.h"
#include "SparkFun_Photon_Weather_Shield_Library.h"
#include "OneWire.h"
#include <Wire.h> //I2C needed for sensors
#include "SparkFunMAX17043.h"

/**************************
Big board settings
*************************/
// Setup the antena on the wifi
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
// Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barometric sensor
Weather sensor;

/**************************
Sensor variables
*************************/
// Wind speed
int WSPEED = D3;
long lastWindCheck = 0;
volatile byte windClicks = 0;
volatile long lastWindIRQ = 0;
float curr_wind_speed = 0;

// Wind direction
const byte WDIR = A0;
float windDirection = 0.0;
float windVaneCosTotal = 0.0;
float windVaneSinTotal = 0.0;
unsigned int windVaneReadingCount = 0;

// Rain
int RAINPIN = D2;
volatile unsigned long raintime, rainlast, raininterval;
float rain = 0.0;
volatile unsigned int rainReads = 0;
volatile unsigned long rainTotal = 0.0;
float RAINCLICK = 0.011;

// Air Temperature
float airTemp = 0.0;
int airTempReads = 0;
float airTempRunningVal = 0.0;

// Soil moisture
int SMOISTURE = A1;
int SOILPOWER = D5;
float soilMoisture = 0.0;
float soilMoistureTotal = 0.0;
float soilMoistureReads = 0;

// Soil Temp
int DS18S20_Pin = D4;    // DS18S20 Signal pin on digital 2
OneWire ds(DS18S20_Pin); // on digital pin 2
float soilTemp = 0.0;
float soilTempRunningTotal = 0.0;
int soilTempReads = 0;

// Humidity
float humidity = 0.0;
int humidityReads = 0;
float humidityRunningTotal = 0.0;

// Air pressure
float pascals = 0.0;
int pascalReads = 0;
float pascalRunningTotal = 0.0;

// Publish/time helpers
int debug_publish_window = 12000;
int prod_publish_window = 1800000;
byte minutes;
long lastPrint = 0;
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
unsigned long lastSync = millis();
time32_t now();
//--------------------Setup the sensors -----------------------
void setup()
{
  Serial.begin(9600); // open serial over USB at 9600 baud

  // Initialize the I2C sensors and ping them
  sensor.begin();

  sensor.setModeBarometer(); // Set to Barometer Mode

  // These are additional MPL3115A2 functions that MUST be called for the sensor to work.
  sensor.setOversampleRate(7); // Set Oversample rate

  sensor.enableEventFlags(); // Necessary register calls to enble temp, baro and alt

  pinMode(WSPEED, INPUT_PULLUP);
  pinMode(WDIR, INPUT);

  pinMode(SOILPOWER, OUTPUT);
  digitalWrite(SOILPOWER, LOW);

  pinMode(RAINPIN, INPUT_PULLUP);
  attachInterrupt(RAINPIN, rainIRQ, FALLING);
  attachInterrupt(WSPEED, wspeedIRQ, FALLING);
  interrupts();

  // LIPO battery reading steps
  lipo.begin();
  lipo.quickStart();
  lipo.setThreshold(10);
}

//----------------Main Program Loop and publish-------------------
void loop()
{

  if (millis() - lastSync > ONE_DAY_MILLIS)
  {
    // Request time synchronization from the Particle Device Cloud
    Particle.syncTime();
    lastSync = millis();
  }

  // This math looks at the current time vs the last time a publish happened
  // When this is out in the wild that happens every 10 minutes
  if (millis() - lastPrint > prod_publish_window) 
  {
    updateWeatherValues();
    airTemp = calculateAirTemp();
    humidity = calculateHumidity();
    pascals = calculatePressure();
    windDirection = calculateWindDirection();
    curr_wind_speed = get_wind_speed();
    rain = calculateRain();
    soilMoisture = calculateSoilMoisture();
    soilTemp = calculateSoilTemp();

    // Record when you published
    lastPrint = millis();

    // Publish to Particle
    publishInfo();

    // Use the printInfo() function to print data out to Serial - for debugging only
    // printInfo();
  }

  // @todo - turn off wifi between publishing to save battery
  // See: https://docs.particle.io/reference/device-os/api/wifi/wifi/
  // https://community.particle.io/t/onewire-bus-functionality-quits-working-when-photon-wifi-is-turned-off/54797/8 
  // https://community.particle.io/t/deactivating-wifi-on-photon/40283


  // Add a bit of a gathering delay to help smooth out weird gusts for wind or bumps that cause the rain catch to jitter
  delay(200);
}

void publishInfo()
{
  char buffer[256];
  JSONBufferWriter writer(buffer, sizeof(buffer));

  writer.beginObject();
  writer.name("published").value((int)Time.now());
  writer.name("humidity").value(humidity);
  writer.name("airtemp").value(airTemp);
  writer.name("pressure").value(pascals);
  writer.name("windspeed").value(curr_wind_speed);
  writer.name("winddirection").value(windDirection);
  writer.name("soiltemp").value(soilTemp);
  writer.name("soilmoisture").value(soilMoisture);
  writer.name("rainfall").value(rain);
  writer.name("batterycharge").value(lipo.getSOC());
  writer.endObject();
  writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0;
  Particle.publish("add-weather", buffer, PRIVATE);
}

//---------------------------------------------------------------
void printInfo()
{
  // This function prints the weather data out to the default Serial Port
  Serial.print("Time:");
  Serial.print((int)Time.now());
  Serial.print(" ");

  Serial.print("Temp:");
  Serial.print(airTemp);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.print("%, ");

  // The MPL3115A2 outputs the pressure in Pascals. However, most weather stations
  // report pressure in hectopascals or millibars. Divide by 100 to get a reading
  // more closely resembling what online weather reports may say in hPa or mb.
  // Another common unit for pressure is Inches of Mercury (in.Hg). To convert
  // from mb to in.Hg, use the following formula. P(inHg) = 0.0295300 * P(mb)
  // More info on conversion can be found here:
  // www.srh.noaa.gov/images/epz/wxcalc/pressureConversion.pdf
  Serial.print("Pressure:");
  Serial.print(pascals / 100);
  Serial.print("hPa, ");
  Serial.print((pascals / 100) * 0.0295300);
  Serial.print("in.Hg ");

  Serial.print("Wind Speed: ");
  Serial.print(curr_wind_speed);
  Serial.print("mph ");

  Serial.print("Wind Direction: ");
  Serial.print(windDirection);
  Serial.print("deg ");

  Serial.print("Soil Temp: ");
  Serial.print(soilTemp);
  Serial.print("deg ");

  Serial.print("Soil Moisture: ");
  Serial.print(soilMoisture);

  Serial.print("Rain: ");
  Serial.print(rain);
  Serial.print("in ");
  Serial.println(" ");
}

//-------------------------Get all weather values --------------------------------------
void updateWeatherValues()
{
  // Measure Relative Humidity from the HTU21D or Si7021
  gatherHumidity();

  // Measure Temperature from the HTU21D or Si7021
  gatherAirTemp();

  // Measure Pressure from the MPL3115A2
  gatherPressure();

  // Measure wind direction from vane
  gatherWindDirection();

  // Measure soil temperature
  gatherSoilTemp();

  // Measure soil moisture
  gatherSoilMoisture();
}

/****************Rain Functions ***************************/

void rainIRQ()
// Count rain gauge bucket tips as they occur
{
  raintime = millis();                // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {

    rainReads++;

    rainlast = raintime; // set up for next event
  }
}

float calculateRain()
{
  float result = float(rainReads) * RAINCLICK;
  rainReads = 0;
  return result;
}

/****************Soil temp Functions ***************************/

// https://github.com/sparkfun/simple_sketches/blob/master/DS18B20/DS18B20.ino
float getSoilTemp()
{
  // returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if (!ds.search(addr))
  {
    // no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28)
  {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++)
  { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); // using two's compliment
  float TemperatureSum = tempRead / 16;

  return (TemperatureSum * 18 + 5) / 10 + 32;
}

void gatherSoilTemp()
{
  soilTempRunningTotal = soilTempRunningTotal + getSoilTemp();
  soilTempReads++;
}

float calculateSoilTemp()
{
  float result = soilTempRunningTotal / float(soilTempReads);
  soilTempReads = 0;
  soilTempRunningTotal = 0.0;
  return result;
}

/****************Soil Moisture Functions ***************************/

// https://learn.sparkfun.com/tutorials/soil-moisture-sensor-hookup-guide
float readSoil()
{

  digitalWrite(SOILPOWER, HIGH);         // turn D7 "On"
  delay(10);                             // wait 10 milliseconds
  float soilVal = analogRead(SMOISTURE); // Read the SIG value form sensor
  digitalWrite(SOILPOWER, LOW);          // turn D7 "Off"
  return soilVal;                        // send current moisture value
}

void gatherSoilMoisture()
{
  soilMoistureTotal = soilMoistureTotal + readSoil();
  soilMoistureReads++;
}

float calculateSoilMoisture()
{
  float result = float(soilMoistureTotal) / float(soilMoistureReads);
  soilMoistureReads = 0;
  soilMoistureTotal = 0;
  return result;
}

/****************Air Temperature Functions ***************************/

void gatherAirTemp()
{
  airTempRunningVal = sensor.getTempF() + airTempRunningVal;
  airTempReads++;
}

float calculateAirTemp()
{
  float result = airTempRunningVal / float(airTempReads);
  airTempReads = 0;
  airTempRunningVal = 0.0;
  return result;
}

/****************Humidity Functions ***************************/

void gatherHumidity()
{
  humidityRunningTotal = sensor.getRH() + humidityRunningTotal;
  humidityReads++;
}

float calculateHumidity()
{
  float result = humidityRunningTotal / float(humidityReads);
  humidityReads = 0;
  humidityRunningTotal = 0.0;
  return result;
}

/****************Barometric Pressure Functions ***************************/

void gatherPressure()
{
  pascalRunningTotal = sensor.readPressure() + pascalRunningTotal;
  pascalReads++;
}

float calculatePressure()
{
  float result = pascalRunningTotal / float(pascalReads);
  pascalReads = 0;
  pascalRunningTotal = 0.0;
  return result;
}

/****************Wind Speed Functions ***************************/

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); // Grab the current time
    windClicks++;           // There is 1.492MPH for each click per second.
  }
}

// Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; // 750ms

  deltaTime /= 1000.0; // Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; // 3 / 0.750s = 4

  windClicks = 0; // Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; // 4 * 1.492 = 5.968MPH

  return (windSpeed);
}

/****************Wind Direction Functions ***************************/

void gatherWindDirection()
{
  // Read the wind vane, and update the running average of the two components of the vector
  unsigned int windVaneRaw = analogRead(WDIR);

  float windVaneRadians = lookupRadiansFromRaw(windVaneRaw);
  if (windVaneRadians > 0 && windVaneRadians < 6.14159)
  {
    windVaneCosTotal += cos(windVaneRadians);
    windVaneSinTotal += sin(windVaneRadians);
    windVaneReadingCount++;
  }
  return;
}

float calculateWindDirection()
{
  if (windVaneReadingCount == 0)
  {
    return 0;
  }
  float avgCos = windVaneCosTotal / float(windVaneReadingCount);
  float avgSin = windVaneSinTotal / float(windVaneReadingCount);
  float result = atan(avgSin / avgCos) * 180.0 / 3.14159;
  windVaneCosTotal = 0.0;
  windVaneSinTotal = 0.0;
  windVaneReadingCount = 0;
  // atan can only tell where the angle is within 180 degrees.  Need to look at cos to tell which half of circle we're in
  if (avgCos < 0)
    result += 180.0;
  // atan will return negative angles in the NW quadrant -- push those into positive space.
  if (result < 0)
    result += 360.0;

  return result;
}

float lookupRadiansFromRaw(unsigned int analogRaw)
{
  // The mechanism for reading the weathervane isn't arbitrary, but effectively, we just need to look up which of the 16 positions we're in.
  if (analogRaw >= 2200 && analogRaw < 2400)
    return (3.14); // South
  if (analogRaw >= 2100 && analogRaw < 2200)
    return (3.53); // SSW
  if (analogRaw >= 3200 && analogRaw < 3299)
    return (3.93); // SW
  if (analogRaw >= 3100 && analogRaw < 3200)
    return (4.32); // WSW
  if (analogRaw >= 3890 && analogRaw < 3999)
    return (4.71); // West
  if (analogRaw >= 3700 && analogRaw < 3780)
    return (5.11); // WNW
  if (analogRaw >= 3780 && analogRaw < 3890)
    return (5.50); // NW
  if (analogRaw >= 3400 && analogRaw < 3500)
    return (5.89); // NNW
  if (analogRaw >= 3570 && analogRaw < 3700)
    return (0.00); // North
  if (analogRaw >= 2600 && analogRaw < 2700)
    return (0.39); // NNE
  if (analogRaw >= 2750 && analogRaw < 2850)
    return (0.79); // NE
  if (analogRaw >= 1510 && analogRaw < 1580)
    return (1.18); // ENE
  if (analogRaw >= 1580 && analogRaw < 1650)
    return (1.57); // East
  if (analogRaw >= 1470 && analogRaw < 1510)
    return (1.96); // ESE
  if (analogRaw >= 1900 && analogRaw < 2000)
    return (2.36); // SE
  if (analogRaw >= 1700 && analogRaw < 1750)
    return (2.74); // SSE
  if (analogRaw > 4000)
    return (-1); // Open circuit?  Probably means the sensor is not connected
  return -1;
}