#include <WebSocketsServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebConfig.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <WiFi.h>

// TODO: Check if lever moved from zero position after 20 ms otherwise cancel shift
// -> Almost done, add more config options for tuning

WebSocketsServer webSocket = WebSocketsServer(81);

// MAIN CONFIGURATION
#define loopTime         2    // 500 Hz
#define rpmLoopTime      400  // 2.5 Hz

#define rpmFactor        60 * ((1000 / rpmLoopTime))

// PIN DEFINES
#define GREEN_LED 5
#define RED_LED   6
#define IGN_FET_1 1
#define IGN_FET_2 2
#define PIEZO_PIN 13
#define HALL1_PIN 10 // Hall Sensor 1 (Gearbox Position)
#define HALL2_PIN 11 // Hall Sensor 2 (Shift pressure)
#define RPM_PIN   7

// GLOBAL VARIABLES
static unsigned long lastPulseMillis = 0; // Time of last RPM calculation
static unsigned long lastCutMillis = 0;   // Time of last ignition cut begin
static unsigned long currMillis = 0;      // millis()
static unsigned long lastMillis = 0;      // Last cycle millis() (currMillis - loopTime)
static unsigned int coilsDisabled = 0;    // Is ignition currently disabled?
volatile unsigned long pulseCount = 0;    // Ignition interrupt count since last RPM calculation
static bool waitHyst = false;             // pressure sensor hysteresis
static int currRpmRange = 0;              // 0-4 to choose cutoffTime from array
static float lastRPM = 0;                 // Last calculated RPM

volatile bool redLED = false;             // Used to toggle red LED on RPM pulse
volatile bool greenLED = false;           // Used to indicate position sensor state

static int pressureValue = 0;
static int gearboxValue = 0;

hw_timer_t *Timer0_Cfg = NULL; // LED blink

volatile unsigned long betaRPM = 0;
hw_timer_t *Timer1_Cfg = NULL; // RPM

String params = "["
  "{"
  "'name':'password',"
  "'label':'WiFi password',"
  "'type':"+String(INPUTPASSWORD)+","
  "'default':''"
  "},"
  "{"
  "'name':'enable',"
  "'label':'Enable QS',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'1'"
  "},"
  "{"
  "'name':'trigmode',"
  "'label':'Trigger Mode',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'0','l':'Piezo'},"
  "{'v':'1','l':'Hall'},"
  "{'v':'2','l':'Loadcell'}],"
  "'default':'1'"
  "},"
  "{"
  "'name':'enablemode',"
  "'label':'Enable Mode',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'0','l':'Static (Time)'},"
  "{'v':'1','l':'Hall'}],"
  "'default':'1'"
  "},"
  "{"
  "'name':'invertpressure',"
  "'label':'Invert Pressure Sensor',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
  "},"
  "{"
  "'name':'invertgearbox',"
  "'label':'Invert Gearbox Sensor',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
  "},"
  "{"
  "'name':'cuttime1_1',"
  "'label':'First Cut (2500-4500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'58'"
  "},"
  "{"
  "'name':'cuttime1_2',"
  "'label':'First Cut (4500-6500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'56'"
  "},"
  "{"
  "'name':'cuttime1_3',"
  "'label':'First Cut (6500-8500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'54'"
  "},"
  "{"
  "'name':'cuttime1_4',"
  "'label':'First Cut (8500-10500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'52'"
  "},"
  "{"
  "'name':'cuttime1_5',"
  "'label':'First Cut (10500-12500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'50'"
  "},"
  "{"
  "'name':'cuttime2_1',"
  "'label':'Second Cut (2500-4500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'66'"
  "},"
  "{"
  "'name':'cuttime2_2',"
  "'label':'Second Cut (4500-6500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'64'"
  "},"
  "{"
  "'name':'cuttime2_3',"
  "'label':'Second Cut (6500-8500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'60'"
  "},"
  "{"
  "'name':'cuttime2_4',"
  "'label':'Second Cut (8500-10500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'56'"
  "},"
  "{"
  "'name':'cuttime2_5',"
  "'label':'Second Cut (10500-12500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'54'"
  "},"
  "{"
  "'name':'minrpm',"
  "'label':'Min RPM',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':100,'max':20000,"
  "'default':'2500'"
  "},"
  "{"
  "'name':'maxrpm',"
  "'label':'Max RPM',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':100,'max':20000,"
  "'default':'12500'"
  "},"
  "{"
  "'name':'rpmmult',"
  "'label':'RPM Multiplier',"
  "'type':"+String(INPUTTEXT)+","
  "'default':'1.00'"
  "},"
  "{"
  "'name':'deadtime',"
  "'label':'Dead time (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':5000,"
  "'default':'350'"
  "},"
  "{"
  "'name':'cutsensitivity',"
  "'label':'Cut sensitivity (0-4095)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':5000,"
  "'default':'2500'"
  "},"
  "{"
  "'name':'cuthysteresis',"
  "'label':'Cut Hysteresis (0-4095)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':5000,"
  "'default':'30'"
  "},"
  "{"
  "'name':'hallcenter',"
  "'label':'Hall centered (0-4095)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':5000,"
  "'default':'1800'"
  "},"
  "{"
  "'name':'halltouching',"
  "'label':'Hall dogs touching (0-4095)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':5000,"
  "'default':'2000'"
  "},"
  "{"
  "'name':'hallengaged',"
  "'label':'Hall gear engaged (0-4095)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':5000,"
  "'default':'2200'"
  "},"
  "{"
  "'name':'twosteprpm',"
  "'label':'2 Step RPM (0=OFF)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':20000,"
  "'default':'0'"
  "},"
  "{"
  "'name':'twostephyst',"
  "'label':'2 Step Hyst',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':5000,"
  "'default':'200'"
  "}"
  "]";

AsyncWebServer server(80);
AsyncWebConfig conf;

struct cfgOptions
{
  char ssid[32];
  char password[32];

  bool enable;

  int trigMode;      // 0 piezo, 1 hall, 2 loadcell
  int enableMode;    // 0 static, 1 hall

  bool invertPressure;
  bool invertGearbox;

  int cutTime1[5]; // ms
  int cutTime2[5]; // ms

  int minRPM;          // 1/min
  int maxRPM;          // 1/min

  float rpmMult;       // factor
  int deadTime;        // ms

  int cutSensitivity;
  int cutHysteresis;
  int hallCenter;
  int hallTouching;
  int hallEngaged;

  int twoStepRPM;
  int twoStepHyst;
} cfg;



void transferWebconfToStruct(String results)
{
    cfg.enable = conf.getBool("enable");

    cfg.trigMode = conf.getString("trigmode").toInt();
    cfg.enableMode = conf.getString("enablemode").toInt();

    cfg.invertPressure = conf.getBool("invertpressure");
    cfg.invertGearbox = conf.getBool("invertgearbox");

    cfg.cutTime1[0] = conf.getInt("cuttime1_1");
    cfg.cutTime1[1] = conf.getInt("cuttime1_2");
    cfg.cutTime1[2] = conf.getInt("cuttime1_3");
    cfg.cutTime1[3] = conf.getInt("cuttime1_4");
    cfg.cutTime1[4] = conf.getInt("cuttime1_5");

    cfg.cutTime2[0] = conf.getInt("cuttime2_1");
    cfg.cutTime2[1] = conf.getInt("cuttime2_2");
    cfg.cutTime2[2] = conf.getInt("cuttime2_3");
    cfg.cutTime2[3] = conf.getInt("cuttime2_4");
    cfg.cutTime2[4] = conf.getInt("cuttime2_5");

    cfg.minRPM = conf.getInt("minrpm");
    cfg.maxRPM = conf.getInt("maxrpm");

    cfg.rpmMult = conf.getFloat("rpmmult");
    cfg.deadTime = conf.getInt("deadtime");

    cfg.cutSensitivity = conf.getInt("cutsensitivity");
    cfg.cutHysteresis = conf.getInt("cuthysteresis");
    cfg.hallCenter = conf.getInt("hallcenter");
    cfg.hallTouching = conf.getInt("halltouching");
    cfg.hallEngaged = conf.getInt("hallengaged");

    cfg.twoStepRPM = conf.getInt("twosteprpm");
    cfg.twoStepHyst = conf.getInt("twostephyst");
}

void handleRoot(AsyncWebServerRequest *request)
{
  conf.handleFormRequest(request);
  /*
  if (request->hasParam("SAVE"))
  {
    transferWebconfToStruct();
  }*/
}



// Create a task that will be executed on Core 0 (instead of 1)
TaskHandle_t Task1;
void push_debug(void *parameters)
{
  for (;;)
  {
    String broadcastString = String(pressureValue) + "," + String(gearboxValue) + "," + String(lastRPM) + "," + String(betaRPM);

    webSocket.loop();
    webSocket.broadcastTXT(broadcastString);

    delay(50);
  }
}

// Interrupt Service Routine (ISR) for the timer
void IRAM_ATTR Timer0_ISR()
{
  greenLED = !greenLED; // Toggle the LED state
  digitalWrite(GREEN_LED, greenLED); // Set the LED pin
}

// Timer configuration by ChatGPT lol
void setupTimer0(int frequency)
{
  // Detach timer to avoid conflicts
  if (Timer0_Cfg != NULL)
  {
    timerDetachInterrupt(Timer0_Cfg);
    timerEnd(Timer0_Cfg);
  }

  // Initialize timer (timer number 0, prescaler = 80, frequency in Hz)
  Timer0_Cfg = timerBegin(0, 80, true); // Prescaler of 80 gives 1us resolution

  // Set the compare value based on the desired frequency
  int compareValue = 1000000 / (frequency * 2); // *2 for toggling on both compare events

  // Set the timer to trigger interrupt at compare value
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);  // ISR on compare match
  timerAlarmWrite(Timer0_Cfg, compareValue, true);   // Set compare value and auto-reload
  timerAlarmEnable(Timer0_Cfg);                      // Enable timer alarm (interrupt)
}



void disable_ign()
{
  digitalWrite(IGN_FET_1, LOW);
  digitalWrite(IGN_FET_2, LOW);
  digitalWrite(GREEN_LED, LOW);
}

void enable_ign_1()
{
  digitalWrite(IGN_FET_1, HIGH);
}

void enable_ign_2()
{
  digitalWrite(IGN_FET_2, HIGH);
  //digitalWrite(GREEN_LED, HIGH);
}

void countPulse()
{
  pulseCount++;
  digitalWrite(RED_LED, (redLED ? HIGH : LOW));
  redLED = !redLED;

  betaRPM = 60000000 / timerReadMicros(Timer1_Cfg);
  timerWrite(Timer1_Cfg, 0);
}



void setup()
{
  pinMode(GREEN_LED, OUTPUT); // Green LED
  pinMode(RED_LED, OUTPUT); // Red LED
  pinMode(IGN_FET_1, OUTPUT); // Ign Fet 1
  pinMode(IGN_FET_2, OUTPUT); // Ign Fet 2

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(IGN_FET_1, HIGH);
  digitalWrite(IGN_FET_2, HIGH);

  pinMode(PIEZO_PIN, INPUT); // Piezo Sensor
  pinMode(HALL1_PIN, INPUT); // Hall Sensor 1 (Gearbox Position)
  pinMode(HALL2_PIN, INPUT); // Hall Sensor 2 (Shift Pressure)

  // Interrupt routine for ingition pulse counting / RPM calculation
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(RPM_PIN, countPulse, RISING);

  // Timer 1 initialisieren (Timer 1, Prescaler = 80 -> 1µs Auflösung)
  Timer1_Cfg = timerBegin(1, 80, true); // Verwende Timer 1 mit 1 µs Auflösung

  conf.setDescription(params);
  conf.readConfig();
  transferWebconfToStruct("");

  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_7dBm);
  WiFi.softAP(conf.getApName(), conf.values[0].c_str());

  server.on("/", handleRoot);

  server.on("/debug.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/debug.html", "text/html");
  });

  conf.registerOnSave(transferWebconfToStruct);

  server.begin();
  webSocket.begin();

  setupTimer0(5);

  xTaskCreatePinnedToCore(push_debug, "CPU_0", 10000, NULL, 1, &Task1, 0);
}

void loop()
{
  currMillis = millis();

  if (currMillis >= (lastMillis + loopTime))
  {
    switch (cfg.trigMode)
    {
      case 0: // Piezo
        pressureValue = analogRead(PIEZO_PIN);
        break;

      case 1: // Hall
        pressureValue = analogRead(HALL2_PIN);
        break;

      case 2: // Loadcell
        pressureValue = 0.f;
        break;
    }

    gearboxValue = analogRead(HALL1_PIN);

    if (cfg.invertPressure)
      pressureValue = 4095 - pressureValue;

    if (cfg.invertGearbox)
      gearboxValue = 4095 - gearboxValue;

    lastMillis = currMillis;
  }

  // Skip all quickshifter features, only 2-step remains
  if (cfg.enable)
  {
    /* ######################### Shift pressure detection and ignition cut ######################### */
    // Disable ignition when pressure is sensed, sufficient RPM, deadTime passed and pressure was low before (hysteresis prevents continuous triggers when holding shift lever up in top gear)
    if (lastRPM >= cfg.minRPM && coilsDisabled == 0 && waitHyst == false && pressureValue >= cfg.cutSensitivity && currMillis >= (lastCutMillis + cfg.deadTime))
    {
      // Don't trigger if gear lever has already moved beyond claw on claw position (e.g clutch pulled and upshifting)
      if (!(cfg.enableMode == 1 && gearboxValue >= cfg.hallTouching))
      {
        disable_ign();
        coilsDisabled = 2;
        waitHyst = true;
  
        currRpmRange = map(lastRPM, cfg.minRPM, cfg.maxRPM, 0, 4);
  
        lastCutMillis = currMillis;
      }
    }
  
    // pressure sensor value went below hysteresis, re-enable triggering
    if (pressureValue < (cfg.cutSensitivity - cfg.cutHysteresis))
      waitHyst = false;
  
    // Enable ignition 1 when dog rings touching or time 1 passed
    if (coilsDisabled == 2)
    {
      if ((cfg.enableMode == 0 && currMillis >= (lastCutMillis + cfg.cutTime1[currRpmRange])) ||
           (cfg.enableMode == 1 && gearboxValue >= cfg.hallTouching))
      {
        enable_ign_1();
        coilsDisabled = 1;
      }
    }
  
    // Enable ignition 2 when gear fully engaged or time 2 passed
    if (coilsDisabled == 1)
    {
      if ((cfg.enableMode == 0 && currMillis >= (lastCutMillis + cfg.cutTime2[currRpmRange])) ||
          (cfg.enableMode == 1 && gearboxValue >= cfg.hallEngaged))
      {
        enable_ign_2();
        coilsDisabled = 0;
      }
    }
  
    // Enable ignition if gear lever hasn't left center position (upshifting in highest gear)
//    if (coilsDisabled > 0 && (gearboxValue <= cfg.hallCenter + abs(cfg.hallTouching - cfg.hallCenter)/2) && currMillis >= (lastCutMillis + 30))
//    {
//      enable_ign_1();
//      enable_ign_2();
//      coilsDisabled = 0;
//    }
  
    //Safety feature to prevent engine shutting off in case of an issue
    if (coilsDisabled == 2 && currMillis >= (lastCutMillis + 250))
    {
      enable_ign_1();
      coilsDisabled = 1;
    }
    if (coilsDisabled == 1 && currMillis >= (lastCutMillis + 260))
    {
      enable_ign_2();
      coilsDisabled = 0;
    }
  }

  /* ######################### Two Step Rev Limiter ######################### */
  // (just for fun, WIP)
  if (cfg.twoStepRPM > 0 && !cfg.enable)
  {
    if (coilsDisabled == 0 && betaRPM > cfg.twoStepRPM)
    {
      disable_ign();
      coilsDisabled = 2;
      lastCutMillis = currMillis;
    }
    else if (coilsDisabled > 0 && currMillis >= (lastCutMillis + cfg.twoStepHyst))
    {
      enable_ign_1();
      enable_ign_2();
      coilsDisabled = 0;
    }
  }


  /* ######################### RPM Calculation ######################### */
  // Calculate RPM from pulseCount
  if (currMillis >= (lastPulseMillis + rpmLoopTime))
  {
    noInterrupts();
    lastRPM = (pulseCount * rpmFactor * cfg.rpmMult);
    pulseCount = 0;
    interrupts();

    lastPulseMillis = currMillis;
  }
}
