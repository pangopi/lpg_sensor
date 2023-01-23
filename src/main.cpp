/*
  LPG detection sensor
  Author: J. Peterse
  For Arduino pro mini (ATMEGA328P). Possilby works on other boards as well, but
  not tested.

  Based on code found on the net but I'm not able to find it again. Heavily
  modified. Uses a temperature and humidity sensor to normalise the
  measurements. Added a watchdog and detection of the presence of the gas
  sensor. When the DHT temp/hum sensor is not found or in another way faulty a
  gas calculation is still performed and the alarm will sound.

  Features:
  - Measures the presence of LPG in ppm
  - Sound alarm when ppm threshold reached
  - Outputs a TTL (5V) signal when gas threshols value is reached to communicate
  with other sensors
  - Green LED to indicate sensor is working and taking samples
  - Red LED to indicate errors
  - When an error occurs that prevents LPG measurements, the green LED will be
  switched off, otherwise the green light will stay lit indicating valid samples
  are still being taken.
  - Watchdog timer (500ms)
  - Chirp alarm and blink error LED on following errors:
    - Gas sensor not found
    - DHT temp/hum sensor not found

  Operation indicators:
  Green  Red   Alarm       State
  Normal operational states
  On     Off   None         Normal operation and taking and analysing samples
  Slow   Off   None         Startup: Performing warmup and calibration
  Alarm states
  On     On    Fast         Gas over threshold detected
  On     Fast  Chirp        Gas detected but under threshold
  Error states
  Off    On    Chirp        Gas sensor not detected or faulty
  On     Slow  None         Temp/Hum sensor faulty (not critical)

*/

/*

  MQ-6 LPG (propane, butane) Gas Sensor
  
  Sensor must be 'burned-in' for 24 hours prior to general use.
  
  After burn-in, regular use of the sensor consists of 
  calibration in air without the presence of LPG.  
  Therafter, the sensor needs to be warmed for 30 sec
  prior to taking a reading.  
  
  This sketch checks for the presense of LPG constantly.  
  (Note that if you only check say every 5 minutes, then you
  may miss an incident).  A total of 500 samples are measured
  from the sample and then the average is used.  This sampling
  takes approximately 500 ms to complete.
  
  If the LPG threshold of 2000 ppm is exceeded, an alarm condition is
  set and it persists until the device is reset.  
  
  The lower explosive limit for LPG is 1900 ppm for butane, and
  2000 ppm for propane.  Human asphyxia occurs at 19000 ppm.
  
  The MQ-6 sensor heater is constantly on.  This heater consume a 
  about 150 mA of current.
*/

#include <Arduino.h>
#include <avr/wdt.h>

#include "LEDBlinker.h"

#define VERSION = 1.1
// #define SERIAL_DEBUG_DISABLED

#define GREEN_LED 5
#define RED_LED 4
#define BUZZER_PIN 3
#define MQ6_PIN A1
#define NPN_PIN 7
#define DHT_PIN 6
#define DIGITAL_OUT 12
#define WARMUP_TIME_MS 90e3

boolean alarmLPG = false;
boolean err_no_sensor = false;
boolean lower_threshold = false;
//  The lower explosive limit for LPG is 1900 ppm for butane, and
//  2000 ppm for propane.
const unsigned int LPG_threshold = 1900;
// Set a lower thershold where the alarm doesn't sound but a chirp is given
const unsigned int LPG_lower_threshold = 400;
float RL = 20.0;  //  load resistor value in k ohms
float Ro = 10.0;  //  default value 10 k ohms.  Revised during calibration.
const float Ro_clean_air_factor = 10.0;
const float vMin_threshold = 100;  // Minimum voltage (in mV) of the sensor used
                                   // in detecting if the sensor is connected

unsigned int LPG_ppm = 0;

LEDBlinker greenLED(GREEN_LED, 0, 0);
LEDBlinker redLED(RED_LED, 0, 0);
LEDBlinker buzzer(BUZZER_PIN, 0, 0);

////////////////////////////////////////////////////////////////////////
// DHT humidity/temperature sensor
#include "DHT.h"
#include "DHT_U.h"
boolean errDHT11 = false;
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11     // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
DHT dht(DHT_PIN, DHTTYPE);
////////////////////////////////////////////////////////////////////////
// With sensor holes facing you:
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

float mV = 0.0;
unsigned long samples = 0;
const int maxSamples = 500;

float RsRoAtAmbientTo20C65RH(float RsRo_atAmb, float ambTemp, float ambRH) {
  //  Using the datasheet for MQ-6 sensor, derive Rs/Ro values 
  //  from - 10 to 50 C and 33, 65, and 85 % relative humidity.
  //  For the measured Rs/Ro, use linear interpolation to calculate the
  //  standard Rs/Ro values for the measured ambient temperature and RH.
  //  Next, calculate a correction factor from the standard Rs/Ro at ambient
  //  temp and RH relative to standard Rs/Ro at 20C and 65 RH.  
  //  Apply this correction factor to the measured Rs/Ro value and return the
  //  corrected value.  This corrected value may then be used against the Rs/Ro
  //  Rs/Ro vs LPG concentration (ppm) chart to estimate the concentration of PPG.
  
  //  Calc RsRo values at ambTemp & 33% RH, 65% and 85% RH
  float RsRo_at_ambTemp_33RH = -0.00000321 * pow(ambTemp, 3) +
                               0.000315 * pow(ambTemp, 2) - 0.0129 * ambTemp +
                               1.1568;
  float RsRo_at_ambTemp_85RH = -0.00000358 * pow(ambTemp, 3) +
                               0.000298 * pow(ambTemp, 2) - 0.0102 * ambTemp +
                               0.9642;
  // float RsRo_at_65RH = ((65.0-33.0)/(85.0-65.0));
  float RsRo_at_ambTemp_65RH =
      ((65.0 - 33.0) / (85.0 - 33.0) *
           (RsRo_at_ambTemp_85RH - RsRo_at_ambTemp_33RH) +
       RsRo_at_ambTemp_33RH) *
      1.102;
  //  Linear interpolate to get the RsRo at the ambient RH value (ambRH).
  float RsRo_at_ambTemp_ambRH;
  if (ambRH < 65.0) {
    RsRo_at_ambTemp_ambRH = (ambRH - 33.0) / (65.0 - 33.0) *
                                (RsRo_at_ambTemp_65RH - RsRo_at_ambTemp_33RH) +
                            RsRo_at_ambTemp_33RH;
  } else {
    // ambRH > 65.0
    RsRo_at_ambTemp_ambRH = (ambRH - 65.0) / (85.0 - 65.0) *
                                (RsRo_at_ambTemp_85RH - RsRo_at_ambTemp_65RH) +
                            RsRo_at_ambTemp_65RH;
  }
  //  Calc the correction factor to bring RsRo at ambient temp & RH to 20 C and
  //  65% RH.
  const float refRsRo_at_20C65RH = 1.00;
  float RsRoCorrPct =
      1 + (refRsRo_at_20C65RH - RsRo_at_ambTemp_ambRH) / refRsRo_at_20C65RH;
  //  Calculate what the measured RsRo at ambient conditions would be corrected
  //  to the conditions for 20 C and 65% RH.
  float measured_RsRo_at_20C65RH = RsRoCorrPct * RsRo_atAmb;
  return measured_RsRo_at_20C65RH;
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

float CalcRsFromVo(float Vo) {
  //  Vo = sensor output voltage in mV.
  //  VRef = supply voltage, 5000 mV
  //  RL = load resistor in k ohms
  //  The equation Rs = (Vc - Vo)*(RL/Vo)
  //  is derived from the voltage divider
  //  principle:  Vo = RL * Vc (Rs + RL)
  //
  //  Note.  Alternatively you could calc
  //         Rs from ADC value using
  //         Rs = RL * (1024 - ADC) / ADC
  float Rs = (readVcc() - Vo) * (RL / Vo);  
  return Rs;
}

unsigned int GetLpgPpmForRatioRsRo(float RsRo_ratio) {
  //  If you extract the data points from the LPG concentration
  //  versus Rs/Ro chart in the datasheet, plot the points,
  //  fit a polynomial curve to the points, you come up with the equation
  //  for the curve of:  Rs/Ro = 18.446 * (LPG ppm) ^ -0.421
  //  This equation is valid for ambient conditions of 20 C and 65% RH.
  //  Solving for the concentration of LPG you get:
  //    LPG ppm = [(Rs/Ro)/18.446]^(1/-0.421)
  float ppm;
  ppm = pow((RsRo_ratio/18.446), (1/-0.421));
  return (unsigned int) ppm;
}

float Get_mVfromADC(byte AnalogPin) {
    // read Vcc from the internal 1.1V reference
    double Vcc = readVcc(); // in mV
    // read the value from the sensor:
    int ADCval = analogRead(AnalogPin);  
    // It takes about 100 microseconds (0.0001 s) to read an analog input
    delay(1);
    //  Voltage at pin in milliVolts = (reading from ADC) * (5000/1024) 
    //float mV = ADCval * (vRef / 1024.0);
    float mV = (ADCval / 1024.0) * Vcc;
    return mV;
}

bool check_led_connected(int pin) {
  // Checks if a led is connected
  // LED connected to ground with a resistor to the I/O pin
  pinMode(pin, INPUT_PULLUP);
  int connected = digitalRead(pin);
  pinMode(pin, OUTPUT);
  if (connected == HIGH){
    return false;
  }
  return true;
}


void setup() {

  // Disable WDT at startup
  wdt_disable(); 

#ifndef SERIAL_DEBUG_DISABLED
  Serial.begin(9600);
  Serial.print("Starting ... ");
#endif

  //  DEFAULT: analog reference of 5 volts on 5V Arduino boards 
  analogReference(DEFAULT);
  delay(1);
  pinMode(MQ6_PIN, INPUT); 
  delay(1);
  pinMode(NPN_PIN, OUTPUT);
  delay(1);
  
  greenLED.setHigh();
  redLED.setHigh();
  tone(BUZZER_PIN, 880, 800);


#ifndef SERIAL_DEBUG_DISABLED
  Serial.print("DHT");
  Serial.print(DHTTYPE);
  Serial.println(" setup");
#endif
  dht.begin();
  //  Get the ambient conditions (deg C & relative humidity) from DHT11
  float ambRH = dht.readHumidity();
  float ambTemp = dht.readTemperature();
  // check if returns are valid, if they are NaN (not a number) then something
  // went wrong!
  if (isnan(ambTemp) || isnan(ambRH)) {
#ifndef SERIAL_DEBUG_DISABLED
    Serial.println("Failed to read from DHT");
    Serial.println("  ");
#endif
    errDHT11 = true;
  } else {
    //  DHT ok, .. proceed.
#ifndef SERIAL_DEBUG_DISABLED
    Serial.print("ambTemp = ");
    Serial.print(ambTemp);
    Serial.println(" deg C");
    Serial.print("ambRH = ");
    Serial.print(ambRH);
    Serial.println("% ");
#endif
  }

#ifndef SERIAL_DEBUG_DISABLED
  Serial.println("Calibrating MQ-6 LPG sensor in clean air.");
  Serial.print("Warming up sensor for ");
  Serial.print(WARMUP_TIME_MS / 1000);
  Serial.print(" seconds...");
  digitalWrite(NPN_PIN, HIGH);
#endif

  unsigned long warmupTime = millis() + WARMUP_TIME_MS;
  //unsigned long warmupTime = millis() + 5e3; // 5 seconds (for testing)
  greenLED.setInterval(200, 800);
  redLED.setLow();
  while(millis() < warmupTime) {
    greenLED.tick();
    wdt_reset();
  }
#ifndef SERIAL_DEBUG_DISABLED
  Serial.println(" Warmup complete. Starting calibration ...");
#endif

  // Get the Vcc accurately
  double Vcc = readVcc()/1000.0;
#ifndef SERIAL_DEBUG_DISABLED
  Serial.print("Vcc measured at: ");
  Serial.print(Vcc);
  Serial.println(" V");
#endif

  //  take a reading..
  for(int i = 30; i > 0; i--){
    greenLED.tick();
    mV += Get_mVfromADC(MQ6_PIN);
    samples += 1;
    wdt_reset();
  }

  mV = mV / (float) samples;
  #ifndef SERIAL_DEBUG_DISABLED
  Serial.print("  avg A");
  Serial.print(MQ6_PIN);
  Serial.print(" for ");
  Serial.print(samples);
  Serial.print(" samples = ");
  Serial.print(mV);
  Serial.println(" mV");
  Serial.print("  Rs = ");
  Serial.println(CalcRsFromVo(mV));
  #endif
  //  Conv output to Ro
  //  Ro = calibration factor for measurement in clean air.
  //  Ro = ((vRef - mV) * RL) / (mV * Ro_clean_air_factor);
  //  Hereafter, measure the sensor output, convert to Rs, and
  //  then calculate Rs/Ro using: Rs = ((Vc-Vo)*RL) / Vo
  Ro = CalcRsFromVo(mV) / Ro_clean_air_factor;
  #ifndef SERIAL_DEBUG_DISABLED
  Serial.print("  Ro = ");
  Serial.println(Ro);
  Serial.println("Sensor calibration in clean air complete.");
  Serial.println("Setup complete. Constantly monitoring for LPG...");
  #endif
  samples = 0;
  mV = 0.0;

  tone(BUZZER_PIN, 880, 400);
  // All setup steps are complete, enable WatchDogTimer
  #ifdef SERIAL_DEBUG_DISABLED
  wdt_enable(WDTO_500MS);
  #else
  wdt_enable(WDTO_2S);
  wdt_reset();
  #endif
}

void loop() {

  wdt_reset();
  
  if (samples < maxSamples) {
    // Collect a new sample
    mV += Get_mVfromADC(MQ6_PIN);
    samples += 1;
    // Validate sample
    // If the sensor is not plugged in the voltage will be very low
    if (mV < vMin_threshold) {
      // Sensor probably not connected
      err_no_sensor = true;
      //samples = 0;
    } else {
      // Measurement succesful
      err_no_sensor = false;
    }
    
  } else {
    // Enough samples to do a calculation
    mV = mV / (float) samples;
    #ifndef SERIAL_DEBUG_DISABLED
    Serial.print("Gas sensor reading: ");
    Serial.print(mV);
    Serial.print(" mV (");
    Serial.print(samples);
    Serial.print(" samples) Recalibrated Ro would be: ");
    Serial.println(CalcRsFromVo(mV) / Ro_clean_air_factor);
    #endif
    //  Conv output to Ro
    //  Calculate Rs from the measured sensor voltage output
    //  using: Rs = (Vc-Vo)* (RL / Vo)
    //float Rs = (vRef - mV) * (RL / mV);
    float Rs = CalcRsFromVo(mV);
    //  Calculate Rs/Ro
    float RsRo_atAmb = Rs / Ro;
    //  Get the ambient conditions (deg C & relative humidity) from DHT
    float ambRH = dht.readHumidity();
    float ambTemp = dht.readTemperature();
    #ifndef SERIAL_DEBUG_DISABLED
    Serial.print("Rs=");
    Serial.print(Rs);
    Serial.print(", Ro=");
    Serial.print(Ro);
    Serial.print(", Ratio: ");
    Serial.print(RsRo_atAmb);
    #endif

    // check if returns are valid, if they are NaN (not a number) then something
    // went wrong!
    if (isnan(ambTemp) || isnan(ambRH)) {
      #ifndef SERIAL_DEBUG_DISABLED
      Serial.println(" - Failed to read from DHT.");
      #endif
      errDHT11 = true;
      // Establish a reading without correcting for temperature and humidity
      // Assume 20 degrees and 65% RH (the sensor's standard values)
      LPG_ppm = GetLpgPpmForRatioRsRo(RsRo_atAmb);
      // digitalWrite(pinNPN, LOW);
    } else {
      //  DHT ok, .. proceed.
      errDHT11 = false;
      float measured_RsRo_at_20C65RH =
          RsRoAtAmbientTo20C65RH(RsRo_atAmb, ambTemp, ambRH);

      #ifndef SERIAL_DEBUG_DISABLED
      Serial.print(" -> Corrected to: ");
      Serial.print(measured_RsRo_at_20C65RH);
      Serial.print(" (T=");
      Serial.print(ambTemp);
      Serial.print(" RH=");
      Serial.print(ambRH);
      Serial.println("%)");
      #endif

      //  Calc the measured Rs/Ro (corrected to std 20C and 65% RH) in LPG
      //  concentration (ppm).
      LPG_ppm = GetLpgPpmForRatioRsRo(measured_RsRo_at_20C65RH);
    }

    #ifndef SERIAL_DEBUG_DISABLED
    Serial.print("LPG ppm = ");
    Serial.println(LPG_ppm);
    #endif
    
    if (LPG_ppm > LPG_threshold) {
      alarmLPG = true;
    } else {
      alarmLPG = false;
      // No alarm but check for lower threshold
      if (LPG_ppm > LPG_lower_threshold) {
        lower_threshold = true;
      } else {
        lower_threshold = false;
      }
    }

    samples = 0;
    mV = 0.0;

    // Check if the LEDS are still connected
    // if (!check_led_connected(GREEN_LED)) {
    //   Serial.println("Green LED not connected!");
    // }
    // if (!check_led_connected(RED_LED)) {
    //   Serial.println("Red LED not connected!");
    // }

    if (err_no_sensor) {
      // If not sensor detected then there are no samples taken and so no LPG
      // threshold is exceeded so give the error code (short red blinking with
      // chirp). If the temp/hum sensor is also not detected that error code
      // will repeat after this one.
      #ifndef SERIAL_DEBUG_DISABLED
      Serial.println("Error: No gas sensor found.");
      #endif

      greenLED.setLow();
      redLED.setInterval(500, 1000);
      buzzer.setInterval(500, 1000);
    } else if (alarmLPG == true) {
      // Sound alarm if gas threshold is exceeded
      #ifndef SERIAL_DEBUG_DISABLED
      Serial.println("LPG alarm");
      #endif

      redLED.setHigh();
      greenLED.setHigh();
      buzzer.setInterval(300, 500);

      // Set the output pin high
      digitalWrite(DIGITAL_OUT, HIGH);
    } else if (lower_threshold == true) {
      // LPG lower threshold reached
      #ifndef SERIAL_DEBUG_DISABLED
      Serial.println("LPG lower threshold reached");
      #endif

      redLED.setInterval(150, 2000);
      greenLED.setHigh();
      buzzer.setInterval(150, 2000);
    } else if (errDHT11 == true) {
      // Only give the error that the temp/hum sensor isn't working when there
      // is no LPG detected. Otherwise the LPG alarm is sounded.
      #ifndef SERIAL_DEBUG_DISABLED
      Serial.println("Error: DHT sensor not found");
      #endif

      greenLED.setInterval(1000, 50);
      redLED.setInterval(150, 4900);
      //buzzer.setInterval(50, 5000);
    } else {
      // No alarms or anything just turn on the green light
      greenLED.setHigh();
      redLED.setLow();
      buzzer.setLow();

      // Set the digital output low
      digitalWrite(DIGITAL_OUT, LOW);
    }
  }

  greenLED.tick();
  redLED.tick();
  buzzer.tick();
}