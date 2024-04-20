// CHANGE CONSOLE DEBUG OUTPUT
//#define DEBUG_SENSOR
//#define DEBUG_SENSOR_PLOT
//#define DEBUG_SHIFT
//#define DEBUG_RPM

// QUICKSHIFTER CONFIGURATION
#define loopTime         5    // 200 Hz
#define rpmLoopTime      200  // 5 Hz

#define cutTime1         60    // 3000 - 5375   RPM
#define cutTime2         55    // 5376 - 7750   RPM
#define cutTime3         55    // 7751 - 10125  RPM
#define cutTime4         50    // 10126 - 12500 RPM

#define minRPM           3000  // min. required RPM to allow QS 
#define maxRPM           12500 // Engine RPM for minCutTime (still works above, always cutTimeMin)
#define rpmStep          (maxRPM - minRPM) / 4

#define delayTime        25   // 25 ms (delay after trigger force is reached)
#define deadTime         300  // 300 ms (dead time before next cut)
#define rpmFactor        60 * ((1000 / rpmLoopTime))

// SENSITIVITY VALUES
#define cutSensitivity   240   // min. piezo force threshold

// PIN DEFINES
#define PIEZO_SENSOR     17 // A3  (Piezo Sensor Pin) (A4/18; A5/19 broken)
#define CUT_SIG          10 // D10, PB2 (Ignition cut MOSFET pin)
#define GREEN_LED        3  // D3
//#define RED_LED          2  // D2

// GLOBAL VARIABLES
static unsigned long lastPrintMillis = 0; // Time of last Serial.print for Sensor value
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

static int sensorValue = 0;

// CALCULATE CUTOFF TIME BASED ON ENGINE RPM
/*
int cutoffTime()
{
  if (lastRPM >= minRPM && lastRPM <= (minRPM + rpmStep))
  {
    return cutTime1;
  }
  else if (lastRPM > (minRPM + rpmStep) && lastRPM <= (minRPM + 2*rpmStep))
  {
    return cutTime2;
  }
  else if (lastRPM > (minRPM + 2*rpmStep) && lastRPM <= (minRPM + 3*rpmStep))
  {
    return cutTime3;
  }
  else if (lastRPM > (minRPM + 3*rpmStep))
  {
    return cutTime4;
  }
}
*/

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
  //pinMode(GREEN_LED,    OUTPUT);
  //pinMode(RED_LED,      OUTPUT);
  //pinMode(CUT_SIG,      OUTPUT);
 
  //digitalWrite(GREEN_LED, LOW);
  //digitalWrite(RED_LED,   LOW);
  //digitalWrite(CUT_SIG,   HIGH);

  //pinMode(PIEZO_SENSOR, INPUT);

  DDRD |= (1 << DDD3); // D3 (Green LED) as Output
  DDRB |= (1 << DDB2); // D10 (Ign Cut FET) as Output

  PORTD &= ~(1 << DDD3); // Turn off Green LED
  PORTB |= (1 << DDB2); // Turn on Ignition

  DDRC &= ~(1 << DDC4); // A4 (Piezo) as Input

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
    lastMillis = currMillis;

    if (currMillis >= lastPrintMillis + 35)
    {
      #ifdef DEBUG_SENSOR
        Serial.print(currMillis); Serial.print(" - sensorValue: "); Serial.println(sensorValue);
      #endif

      #ifdef DEBUG_SENSOR_PLOT
        Serial.print("min:-512,max:512,sensorValue:"); Serial.println(sensorValue);
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