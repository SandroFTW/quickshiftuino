// Uncomment to use load cell instead of piezo sensor
//#define LOADCELL

// CHANGE CONSOLE DEBUG OUTPUT
//#define DEBUG_SENSOR_PLOT
//#define DEBUG_SENSOR
  #define DEBUG_SHIFT
//#define DEBUG_RPM

#ifdef LOADCELL
  #include <HX711_ADC.h>
  #include <EEPROM.h>

  #define calAdressEEPROM 0
  #define tareAdressEEPROM 4

  #define HX711_DOUT 4 // MCU -> HX711 Dout Pin
  #define HX711_SCK  5 // MCU -> HX711 SCK Pin
#elif
  #define PIEZO_SENSOR    19 // A5  (Piezo Sensor Pin)
#endif

// QUICKSHIFTER CONFIGURATION
#define loopTime         5    // 200 Hz
#define rpmLoopTime      200  // 5 Hz

#define cutTimeMax       100  // 100 ms at minRPM
#define cutTimeMin       85   // 85 ms at maxRPM

#define minRPM           3000  // min. required RPM to allow QS 
#define maxRPM           12000 // Engine RPM for minCutTime (still works above maxRPM, cutTime is always cutTimeMin)

#define delayTime        25   // 25 ms (delay after trigger force is reached)
#define deadTime         300  // 300 ms (dead time before next cut)
#define rpmFactor        60 * ((1000 / rpmLoopTime))

// SENSITIVITY VALUES
#ifdef LOADCELL
  #define cutSensitivity   5000  // ~ 5000 g on the load cell
#elif
  #define cutSensitivity   200   // min. piezo force threshold
#endif

// PIN DEFINES
#define CUT_SIG   10 // D10 (Ignition cut MOSFET pin)
#define GREEN_LED 3  // D3
#define RED_LED   2  // D2

// GLOBAL VARIABLES
static unsigned long lastPulseMillis = 0; // Time of last RPM calculation
static unsigned long lastCutMillis = 0;   // Time of last shiftQueued
static unsigned long currMillis = 0;      // millis()
static unsigned long lastMillis = 0;      // Last cycle millis() (currMillis - loopTime)
static volatile int pulseCount = 0;       // Ignition interrupt count since last RPM calculation
static bool ignitionDisabled = false;     // Is ignition currently disabled?
static bool shiftCooldown = true;         // Is QS on cooldown after last shift?
static bool shiftQueued = false;          // Is shiftQueued because threshold was reached?
static float cutTime = 0;                 // Calculated cutTime based on RPM
static float lastRPM = 0;                 // Last calculated RPM

#ifdef LOADCELL
  HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);
  static float sensorValue = 0;
#elif
  static int sensorValue = 0;
#endif

// CALCULATE CUTOFF TIME BASED ON ENGINE RPM
int cutoffTime()
{
  if (lastRPM > maxRPM)
  {
    return cutTimeMin; // If RPM is above maxRPM always use cutTimeMin
  }
  else
  {
    return map(lastRPM, minRPM, maxRPM, cutTimeMax, cutTimeMin); // Scales cutoffTime based on RPM (more RPM -> shorter cut)
  }
}

void disable_ign()
{
  //PORTB |= (0 << 2); // Disable Ignition
  //PORTD |= (1 << 3); // Enable Green LED
  digitalWrite(CUT_SIG, LOW);
  digitalWrite(GREEN_LED, HIGH);
}

void enable_ign()
{
  //PORTB |= (1 << 2); // Enable Ignition
  //PORTD |= (0 << 3); // Disable Green LED
  digitalWrite(CUT_SIG, HIGH);
  digitalWrite(GREEN_LED, LOW);
}

void countPulse()
{
  pulseCount++;
}

int updatePiezoValue()
{
  if (currMillis >= (lastMillis + loopTime))
  {
    sensorValue = analogRead(PIEZO_SENSOR) - 512;
    lastMillis = currMillis;

    #ifdef DEBUG_SENSOR
      Serial.print(currMillis); Serial.print(" - sensorValue: "); Serial.println(sensorValue);
    #endif

    #ifdef DEBUG_SENSOR_PLOT
      Serial.print("min:-512,max:512,sensorValue:"); Serial.println(sensorValue);
    #endif
  }
}

float updateLoadcellValue()
{
  if (LoadCell.update())
  {
    sensorValue = LoadCell.getData();
    lastMillis = currMillis;

    #ifdef DEBUG_SENSOR
      Serial.print(currMillis); Serial.print(" - sensorValue: "); Serial.println(sensorValue);
    #endif

    #ifdef DEBUG_SENSOR_PLOT
      Serial.print("min:-20000,max:20000,sensorValue:"); Serial.println(sensorValue);
    #endif
  }
}



void setup()
{
  pinMode(GREEN_LED,    OUTPUT);
  //pinMode(RED_LED,      OUTPUT);
  pinMode(CUT_SIG,      OUTPUT);

  digitalWrite(GREEN_LED, LOW);
  //digitalWrite(RED_LED,   HIGH);
  digitalWrite(CUT_SIG,   HIGH);

  Serial.begin(57600);

  #ifdef LOADCELL
    LoadCell.begin();

    float calibrationValue; // = 75.2 for gramms
    long tareValue;
    EEPROM.get(calAdressEEPROM, calibrationValue);
    EEPROM.get(tareAdressEEPROM, tareValue);

    Serial.print("calValue: "); Serial.print(calibrationValue); Serial.print(" - tareValue: "); Serial.println(tareValue);

    LoadCell.start(stabilizingTime, false); // Set to true to enable tare on power-up

    if (LoadCell.getTareTimeoutFlag()) { Serial.println("MCU>HX711 timeout"); while (1); }
    else { // HX711 connection successful
      LoadCell.setCalFactor(calibrationValue);
      PORTD |= (0 << 3); // Disable Green LED when startup is complete -> digitalWrite(GREEN_LED, LOW);
    }
  #elif
    pinMode(PIEZO_SENSOR, INPUT);
  #endif

  // Interrupt routine for ingition pulse counting / RPM calculation
  /*
  DDRD &= ~(1 << 2); // Set D2 as input pin
  PORTD |= (1 << 2); // Enable internal pullup for D2
  attachInterrupt(0, countPulse, RISING); // D2 as Interrupt
  */
}

void loop()
{
  currMillis = millis();

  if (currMillis >= (lastMillis + loopTime))
  {
    #ifdef LOADCELL
      sensorValue = updateLoadcellValue();
    #elif
      sensorValue = updatePiezoValue();

    #ifdef DEBUG_SENSOR
      Serial.print(currMillis); Serial.print(" - sensorValue: "); Serial.println(sensorValue);
      //Serial.print("min:-512,max:512,sensorValue:"); Serial.println(sensorValue);            // Plot min/max for piezo sensorValue
      //Serial.print("min:-20000,max:20000,sensorValue:"); Serial.println(sensorValue);        // Plot min/max for loadcell sensorValue
    #endif
  }



  /* ######################### Shift pressure detection and ignition Cut ######################### */
  // shiftQueued when threshold is reached and not on cooldown
  if (sensorValue >= cutSensitivity && !shiftQueued && !shiftCooldown)
  {
    if (lastRPM >= minRPM)
    {
      lastCutMillis = currMillis;
      shiftQueued = true;

      #ifdef DEBUG_SHIFT
        Serial.print(currMillis); Serial.print(" - shiftQueued = true - RPM: "); Serial.println(lastRPM);
      #endif
    }
    else
    {
      shiftCooldown = true;

      #ifdef DEBUG_SHIFT
        Serial.print(currMillis); Serial.print(" - RPM too low: "); Serial.println(lastRPM);
      #endif
    }
  }

  // Wait for delayTime if shiftQueued
  if (currMillis >= (lastCutMillis + delayTime) && shiftQueued)
  {
    #ifdef DEBUG_SHIFT
      Serial.print(currMillis); Serial.print(" - shiftQueued and delayTime passed - sensorValue: "); Serial.println(sensorValue);
    #endif

    if (sensorValue >= cutSensitivity)// Check if sensor still above threshold
    {
      disable_ign();

      cutTime = cutoffTime();
      ignitionDisabled = true;
      shiftCooldown = true;
      shiftQueued = false;

      #ifdef DEBUG_SHIFT
        Serial.print(currMillis); Serial.print(" - Shift executed (Ignition Disabled) - Cut Time: "); Serial.println(cutTime);
      #endif
    }
    else // Cancel if below threshold
    {
      shiftQueued = false;

      #ifdef DEBUG_SHIFT
        Serial.print(currMillis); Serial.println(" - Shift canceled (Sensor force insufficient)\n");
      #endif
    }
  }

  // Enable Ignition when cutTime has passed
  if (currMillis >= (lastCutMillis + delayTime + cutTime) && ignitionDisabled)
  {
    enable_ign();
    ignitionDisabled = false;

    #ifdef DEBUG_SHIFT
      Serial.print(currMillis); Serial.println(" - Shift complete (Ignition Enabled)\n");
    #endif
  }

  // Reset cooldown when reset sensor threshold is reached and cutTime + delayTime has passed
  if (sensorValue < cutSensitivity && (currMillis >= (lastCutMillis + delayTime + cutTime + deadTime)) && shiftCooldown)
  {
    shiftCooldown = false;
  }



  /* ######################### RPM Calculation ######################### */
  // Calculate RPM from pulseCount
  if (currMillis >= (lastPulseMillis + rpmLoopTime))
  {
    noInterrupts();
    //lastRPM = (pulseCount * 60 * ((1000 / rpmLoopTime)));
    lastRPM = 7000;
    pulseCount = 0;
    interrupts();

    lastPulseMillis = currMillis;

    #ifdef DEBUG_RPM
      Serial.print(currMillis); Serial.print(" - RPM: "); Serial.println(lastRPM);
    #endif
  }
}
