// CHANGE CONSOLE DEBUG OUTPUT
//#define DEBUG_SENSOR
//#define DEBUG_SENSOR_PLOT
//#define DEBUG_HALL_PLOT
//#define DEBUG_SHIFT
//#define DEBUG_RPM

// Possible BUGS in Hall Mode: - Gear is not engaged anymore after 5ms time has passen -> false neutral
//                             - Engine shuts off because gearSensitivity is never reached (always turn on ignition after cutSafeTime?)

#define USE_HALL

// QUICKSHIFTER CONFIGURATION
#define loopTime         5    // 200 Hz
#define rpmLoopTime      200  // 5 Hz

#define cutTime1         85    // 3000 - 5375   RPM 60
#define cutTime2         80    // 5376 - 7750   RPM 55
#define cutTime3         75    // 7751 - 10125  RPM 55
#define cutTime4         70    // 10126 - 12500 RPM 50
#define cutSafeTime      150

#define minRPM           2500  // min. required RPM to allow QS    3000
#define maxRPM           12500 // Engine RPM for minCutTime (still works above, always cutTimeMin)
#define rpmStep          (maxRPM - minRPM) / 4

#define delayTime        25   // 25 ms (delay after trigger force is reached)
#define deadTime         300  // 300 ms (dead time before next cut)
#define rpmFactor        60 * ((1000 / rpmLoopTime))

// SENSITIVITY VALUES
#define cutSensitivity    220   // min. piezo force threshold
#define cancelSensitivity -40   // threshold to detect downshift direction
#define centerSensitivity 0     // value to sense gear lever in center position
#define gearSensitivity   200   // threshold to sense gear fully engaged
#define centerTolerance   30    // +/- tolerance for center position

// PIN DEFINES
#define PIEZO_SENSOR     17 // A3  (Piezo Sensor Pin) (A4/18; A5/19 broken)
#define HALL_SENSOR      16 // A2  (Hall Sensor Pin)
#define CUT_SIG          10 // D10, PB2 (Ignition cut MOSFET pin)
#define GREEN_LED        3  // D3
//#define RED_LED          2  // D2

// GLOBAL VARIABLES
static unsigned long lastPrintMillis = 0; // Time of last Serial.print for Sensor value
static unsigned long lastPulseMillis = 0; // Time of last RPM calculation
static unsigned long lastCutMillis = 0;   // Time of last shiftQueued
static unsigned long currMillis = 0;      // millis()
static unsigned long lastMillis = 0;      // Last cycle millis() (currMillis - loopTime)
static unsigned long finishedMillis = 0;  // Time when hall sensor detected gear fully engaged
static volatile int pulseCount = 0;       // Ignition interrupt count since last RPM calculation
static bool ignitionDisabled = false;     // Is ignition currently disabled?
static bool shiftCooldown = true;         // Is QS on cooldown after last shift?
static bool shiftQueued = false;          // Is shiftQueued because threshold was reached?
static float cutTime = 0;                 // Calculated cutTime based on RPM
static float lastRPM = 0;                 // Last calculated RPM

static int sensorValue = 0;
static int hallValue = 0;

int cutoffTime() // rpmstep = 2375
{
  int step = (lastRPM - minRPM) / rpmStep;

  if (step == 0)
  {
    return cutTime1;
  }
  else if (step == 1)
  {
    return cutTime2;
  }
  else if (step == 2)
  {
    return cutTime3;
  }
  else if (step >= 3)
  {
    return cutTime4;
  }
}

void disable_ign()
{
  PORTB &= ~(1 << 2);  // Disable Ignition
  PORTD |= (1 << 3); // Enable Green LED

  //digitalWrite(CUT_SIG, LOW);
  //digitalWrite(GREEN_LED, HIGH);
}

void enable_ign()
{
  PORTB |= (1 << 2); // Enable Ignition
  PORTD &= ~(1 << 3);   // Disable Green LED

  //digitalWrite(CUT_SIG, HIGH);
  //digitalWrite(GREEN_LED, LOW);
}

void countPulse()
{
  pulseCount++;
}



void setup()
{
  DDRD |= (1 << DDD3); // D3 (Green LED) as Output
  DDRB |= (1 << DDB2); // D10 (Ign Cut FET) as Output

  PORTD &= ~(1 << DDD3); // Turn off Green LED
  PORTB |= (1 << DDB2); // Turn on Ignition

  DDRC &= ~(1 << DDC4); // A4 (Piezo) as Input
  DDRC &= ~(1 << DDC2); // A2 (Hall) as Input

  Serial.begin(57600); // 115200

  // Interrupt routine for ingition pulse counting / RPM calculation
  DDRD &= ~(1 << 2); // Set D2 as input pin
  PORTD |= (1 << 2); // Enable internal pullup for D2
  attachInterrupt(0, countPulse, RISING); // D2 as Interrupt
}

void loop()
{
  currMillis = millis();

  if (currMillis >= (lastMillis + loopTime))
  {
    sensorValue = analogRead(PIEZO_SENSOR) - 512;
    hallValue = analogRead(HALL_SENSOR) - 512;
    lastMillis = currMillis;

    if (currMillis >= lastPrintMillis + 35)
    {
      #ifdef DEBUG_SENSOR
        Serial.print(currMillis); Serial.print(" - sensorValue: "); Serial.println(sensorValue);
      #endif

      #ifdef DEBUG_SENSOR_PLOT
        Serial.print("min:-512,max:512,sensorValue:"); Serial.println(sensorValue);
      #endif

      #ifdef DEBUG_HALL_PLOT
        Serial.print("min:0,max:1023,hallValue:"); Serial.println(hallValue);
      #endif

      lastPrintMillis = currMillis;
    }
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
      Serial.println("Ignition disabled.");

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

#ifdef USE_HALL
  // Use hall sensor value instead of fixed cutoff time
  if ((hallValue >= gearSensitivity) && ignitionDisabled) // && currMillis >= (lastCutMillis + delayTime + 25)) 
  {
    finishedMillis = currMillis;
  }

  // Add 5ms of delay after hall sensor detected shift complete, just to be safe (or enable instantly when downshift is detected)
  if (((finishedMillis >= (currMillis + 5)) || hallValue <= cancelSensitivity) && ignitionDisabled)
  {
    enable_ign();
    ignitionDisabled = false;
    //Serial.println("Ignition enabled - (hall sensor detected upshift complete");
  }

  // Reset cooldown after hall sensor detected shift lever in center position
  if (sensorValue < cutSensitivity && shiftCooldown)
  {
    if ((hallValue <= (centerSensitivity + hallTolerance) && hallValue >= (centerSensitivity - hallTolerance)) || (currMillis >= (lastCutMillis + delayTime + cutSafeTime)))
    {
      shiftCooldown = false;
    }
  }
#else
  // Enable Ignition when cutTime has passed
  if (currMillis >= (lastCutMillis + delayTime + cutTime) && ignitionDisabled)
  {
    enable_ign();
    ignitionDisabled = false;

    #ifdef DEBUG_SHIFT
      Serial.print(currMillis); Serial.println(" - Shift complete (Ignition Enabled, cutTime passed)\n");
    #endif
  }

  // Reset cooldown when reset sensor threshold is reached and cutTime + delayTime has passed
  if (sensorValue < cutSensitivity && (currMillis >= (lastCutMillis + delayTime + cutTime + deadTime)) && shiftCooldown)
  {
    shiftCooldown = false;
  }
#endif



  /* ######################### RPM Calculation ######################### */
  // Calculate RPM from pulseCount
  if (currMillis >= (lastPulseMillis + rpmLoopTime))
  {
    noInterrupts();
    lastRPM = (pulseCount * 60 * ((1000 / rpmLoopTime)));
    //lastRPM = 6000;
    pulseCount = 0;
    interrupts();

    lastPulseMillis = currMillis;

    #ifdef DEBUG_RPM
      Serial.print(currMillis); Serial.print(" - RPM: "); Serial.println(lastRPM);
    #endif
  }
}