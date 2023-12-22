/******************************************************************************
  SparkFun Photon Weather Shield basic example
  Joel Bartlett @ SparkFun Electronics
  Original Creation Date: May 18, 2015
  Updated August 21, 2015
  This sketch prints the temperature, humidity, and barometric pressure OR
  altitude to the Serial port.

  The library used in this example can be found here:
  https://github.com/sparkfun/SparkFun_Photon_Weather_Shield_Particle_Library

  Hardware Connections:
  This sketch was written specifically for the Photon Weather Shield,
  which connects the HTU21D and MPL3115A2 to the I2C bus by default.
  If you have an HTU21D and/or an MPL3115A2 breakout,	use the following
  hardware setup:
      HTU21D ------------- Photon
      (-) ------------------- GND
      (+) ------------------- 3.3V (VCC)
       CL ------------------- D1/SCL
       DA ------------------- D0/SDA

    MPL3115A2 ------------- Photon
      GND ------------------- GND
      VCC ------------------- 3.3V (VCC)
      SCL ------------------ D1/SCL
      SDA ------------------ D0/SDA

  Development environment specifics:
    IDE: Particle Dev
    Hardware Platform: Particle Photon
                       Particle Core

  This code is beerware; if you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!
  Distributed as-is; no warranty is given.
*******************************************************************************/
/**************************
Helping libraries
*************************/
#include "application.h"
#include "SparkFun_Photon_Weather_Shield_Library.h"
#include "OneWire.h"
#include <Wire.h> //I2C needed for sensors

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
float curr_wind_direction = 0;

// Rain
int RAIN = D2;
volatile unsigned long raintime, rainlast, raininterval, rain;
volatile float dailyrainin;
volatile float rainHour[60];

// Air Temperature
float tempf = 0.0;
int airTempReads = 0;
float airTempRunningVal = 0.0;

// Soil temp and moisture
int SMOISTURE = A1;
int SOILPOWER = D5;
float soilVal = 0;
int DS18S20_Pin = D4;    // DS18S20 Signal pin on digital 2
OneWire ds(DS18S20_Pin); // on digital pin 2
float tempsf = 0;

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
int prod_publish_window = 60000;
byte minutes;
long lastPrint = 0;

//----------------Main Program Loop-------------------
void loop()
{

  // This math looks at the current time vs the last time a publish happened
  if (millis() - lastPrint > debug_publish_window) // Publishes every 12000 milliseconds, or 12 seconds
  {
    calculateAirTemp();
    calculateHumidity();
    calculatePressure();

    // Record when you published
    lastPrint = millis();

    // Use the printInfo() function to print data out to Serial
    printInfo();

    publishInfo();
  }
  // Otherwise, get all of the weather values that we keep running averages for
  else
  {
    updateWeatherValues();
  }
  // Add a bit of a gathering delay to help smooth out weird gusts, etc
  delay(100);
}

//--------------------Setup the sensors -----------------------
void setup()
{
  Serial.begin(9600); // open serial over USB at 9600 baud

  // Make sure your Serial Terminal app is closed before powering your device
  // Now open your Serial Terminal, and hit any key to continue!
  // Serial.println("Press any key to begin");
  // This line pauses the Serial port until a key is pressed
  // while(!Serial.available()) Spark.process();

  // Initialize the I2C sensors and ping them
  sensor.begin();

  /*You can only receive accurate barometric readings or accurate altitude
  readings at a given time, not both at the same time. The following two lines
  tell the sensor what mode to use. You could easily write a function that
  takes a reading in one made and then switches to the other mode to grab that
  reading, resulting in data that contains both accurate altitude and barometric
  readings. For this example, we will only be using the barometer mode. Be sure
  to only uncomment one line at a time. */
  sensor.setModeBarometer(); // Set to Barometer Mode
  // baro.setModeAltimeter();//Set to altimeter Mode

  // These are additional MPL3115A2 functions that MUST be called for the sensor to work.
  sensor.setOversampleRate(7); // Set Oversample rate
  // Call with a rate from 0 to 7. See page 33 for table of ratios.
  // Sets the over sample rate. Datasheet calls for 128 but you can set it
  // from 1 to 128 samples. The higher the oversample rate the greater
  // the time between data samples.

  sensor.enableEventFlags(); // Necessary register calls to enble temp, baro and alt

  pinMode(WSPEED, INPUT_PULLUP);
  pinMode(WDIR, INPUT);

  pinMode(SOILPOWER, OUTPUT);
  digitalWrite(SOILPOWER, LOW);

  pinMode(RAIN, INPUT_PULLUP);
  attachInterrupt(RAIN, rainIRQ, FALLING);
  attachInterrupt(WSPEED, wspeedIRQ, FALLING);
  interrupts();
}

void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis();                // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011;       // Each dump is 0.011" of water
    rainHour[minutes] += 0.011; // Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); // Grab the current time
    windClicks++;           // There is 1.492MPH for each click per second.
  }
}

void publishInfo()
{
  char buffer[256];
  JSONBufferWriter writer(buffer, sizeof(buffer));
  // sprintf( buffer, "%f:%f:%f:%f:%f:%f:%f:%f", humidity, tempf, baroTemp, pascals, curr_wind_speed, curr_wind_direction, tempsf, soilVal );
  writer.beginObject();
  writer.name("humidity").value(humidity);
  writer.name("air-temperature").value(tempf);
  writer.name("pascals").value(pascals);
  writer.name("current-wind-speed").value(curr_wind_speed);
  writer.name("current-wind-direction").value(curr_wind_direction);
  writer.name("soil-temperature").value(tempsf);
  writer.name("soil-moisture").value(soilVal);
  writer.endObject();
  writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0;
  Particle.publish("add-weather", buffer, PRIVATE);
}

// https://github.com/sparkfun/simple_sketches/blob/master/DS18B20/DS18B20.ino
float getTemp()
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

// https://learn.sparkfun.com/tutorials/soil-moisture-sensor-hookup-guide
int readSoil()
{

  digitalWrite(SOILPOWER, HIGH);   // turn D7 "On"
  delay(10);                       // wait 10 milliseconds
  soilVal = analogRead(SMOISTURE); // Read the SIG value form sensor
  digitalWrite(SOILPOWER, LOW);    // turn D7 "Off"
  return soilVal;                  // send current moisture value
}

//---------------------------------------------------------------
void updateWeatherValues()
{
  // Measure Relative Humidity from the HTU21D or Si7021
  gatherHumidity();

  // Measure Temperature from the HTU21D or Si7021
  gatherAirTemp();

  // Measure Pressure from the MPL3115A2
  gatherPressure();

  curr_wind_speed = get_wind_speed();
  int wind_dir = get_wind_direction();

  tempsf = getTemp();

  soilVal = readSoil();
}

void gatherAirTemp()
{
  airTempRunningVal = sensor.getTempF() + airTempRunningVal;
  airTempReads++;
}

void calculateAirTemp()
{
  tempf = airTempRunningVal / float(airTempReads);
  airTempReads = 0;
  airTempRunningVal = 0.0;
}

void gatherHumidity()
{
  humidityRunningTotal = sensor.getRH() + humidityRunningTotal;
  humidityReads++;
}

void calculateHumidity()
{
  humidity = humidityRunningTotal / float(humidityReads);
  humidityReads = 0;
  humidityRunningTotal = 0.0;
}

void gatherPressure()
{
  pascalRunningTotal = sensor.readPressure() + pascalRunningTotal;
  pascalReads++;
}

void calculatePressure()
{
  pascals = pascalRunningTotal / float(pascalReads);
  pascalReads = 0;
  pascalRunningTotal = 0.0;
}

//---------------------------------------------------------------
void printInfo()
{
  // This function prints the weather data out to the default Serial Port

  Serial.print("Temp:");
  Serial.print(tempf);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.print("%, ");

  Serial.print("Pressure:");
  Serial.print(pascals / 100);
  Serial.print("hPa, ");
  Serial.print((pascals / 100) * 0.0295300);
  Serial.print("in.Hg ");

  Serial.print("Wind Speed: ");
  Serial.print(curr_wind_speed);
  Serial.print("mph ");

  Serial.print("Wind Direction: ");
  Serial.print(curr_wind_direction);
  Serial.print("deg ");

  Serial.print("Soil Temp: ");
  Serial.print(tempsf);
  Serial.print("deg ");

  Serial.print("Soil Moisture: ");
  Serial.print(soilVal);
  Serial.println(" ");

  // The MPL3115A2 outputs the pressure in Pascals. However, most weather stations
  // report pressure in hectopascals or millibars. Divide by 100 to get a reading
  // more closely resembling what online weather reports may say in hPa or mb.
  // Another common unit for pressure is Inches of Mercury (in.Hg). To convert
  // from mb to in.Hg, use the following formula. P(inHg) = 0.0295300 * P(mb)
  // More info on conversion can be found here:
  // www.srh.noaa.gov/images/epz/wxcalc/pressureConversion.pdf

  // If in altitude mode, print with these lines
  // Serial.print("Altitude:");
  // Serial.print(altf);
  // Serial.println("ft.");
}

/****************Wind Functions***************************/

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

// Takes an average of readings on a given pin
// Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

int get_wind_direction()
// read the wind direction sensor, return heading in degrees
{
  unsigned int adc;

  adc = averageAnalogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  bool wind_dir_debug = FALSE;

  if (wind_dir_debug)
  {
    Serial.println(adc);
  }

  if (adc > 1940 && adc < 2290)
  {
    curr_wind_direction = 0;
    if (wind_dir_debug)
    {
      Serial.println("N");
    }
  }

  else if (adc > 3170 && adc < 3260)
  {
    curr_wind_direction = 45;
    if (wind_dir_debug)
    {
      Serial.println("NE");
    }
  }

  else if (adc > 3900 && adc < 3950)
  {
    curr_wind_direction = 90;
    if (wind_dir_debug)
    {
      Serial.println("E");
    }
  }

  else if (adc > 3820 && adc < 3840)
  {
    curr_wind_direction = 135;
    if (wind_dir_debug)
    {
      Serial.println("SE");
    }
  }

  else if (adc > 3400 && adc < 3620)
  {
    curr_wind_direction = 180;
    if (wind_dir_debug)
    {
      Serial.println("S");
    }
  }

  else if (adc > 2640 && adc < 2810)
  {
    curr_wind_direction = 225;
    if (wind_dir_debug)
    {
      Serial.println("SW");
    }
  }

  else if (adc > 1490 && adc < 1610)
  {
    curr_wind_direction = 270;
    if (wind_dir_debug)
    {
      Serial.println("W");
    }
  }

  else if (adc > 1700 && adc < 1940)
  {
    curr_wind_direction = 315;
    if (wind_dir_debug)
    {
      Serial.println("NW");
    }
  }

  return (curr_wind_direction);
}
