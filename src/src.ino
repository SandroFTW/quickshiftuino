#include <WebSocketsServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebConfig.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <WiFi.h>

// TODO: Check if lever moved from zero position after 20 ms otherwise cancel shift
// -> Almost done, add more config options for tuning

// waitHyst feature not compatible with piezo sensor

// Make LED functions more useful (green led shift pressure indicator, red led rpm)

WebSocketsServer webSocket = WebSocketsServer(81);

// MAIN CONFIGURATION
#define loopTime         2    // 500 Hz

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
static unsigned long lastCutMillis = 0;   // Time of last ignition cut begin
static unsigned long currMillis = 0;      // millis()
static unsigned long lastMillis = 0;      // Last cycle millis() (currMillis - loopTime)
static unsigned int coilsDisabled = 0;    // Is ignition currently disabled?
static bool waitHyst = true;              // pressure sensor hysteresis
static int currRpmRange = 0;              // 0-4 to choose cutoffTime from array
volatile bool redLED = false;             // Used to toggle red LED on RPM pulse
volatile bool greenLED = false;           // Used to indicate position sensor state

static int pressureValue = 0;
static int gearboxValue = 0;

hw_timer_t *Timer0_Cfg = NULL; // LED blink
volatile int cycleCounter = 0;

volatile float lastRPM = 0;
volatile float quickRPM = 0;
volatile uint8_t average_count = 0;
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
  "{'v':'1','l':'Piezo (Inverted)'},"
  "{'v':'2','l':'Hall 2'},"
  "{'v':'3','l':'Hall 2 (Inverted)'},"
  "{'v':'4','l':'Loadcell'}],"
  "'default':'2'"
  "},"
  "{"
  "'name':'enablemode',"
  "'label':'Enable Mode',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'0','l':'Static (Time)'},"
  "{'v':'1','l':'Hall 1'},"
  "{'v':'2','l':'Hall 1 (Inverted)'}],"
  "'default':'1'"
  "},"
  "{"
  "'name':'cuttime_1',"
  "'label':'First Cut (2500-4500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'66'"
  "},"
  "{"
  "'name':'cuttime_2',"
  "'label':'First Cut (4500-6500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'64'"
  "},"
  "{"
  "'name':'cuttime_3',"
  "'label':'First Cut (6500-8500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'64'"
  "},"
  "{"
  "'name':'cuttime_4',"
  "'label':'First Cut (8500-10500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'62'"
  "},"
  "{"
  "'name':'cuttime_5',"
  "'label':'First Cut (10500-12500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':500,"
  "'default':'60'"
  "},"
  "{"
  "'name':'cutsmooth',"
  "'label':'Cut Smoothness',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'0','l':'Very Hard (0 ms)'},"
  "{'v':'2','l':'Hard (4 ms)'},"
  "{'v':'4','l':'Medium (8 ms)'},"
  "{'v':'6','l':'Soft (12 ms)'},"
  "{'v':'8','l':'Very Soft (16 ms)'}],"
  "'default':'8'"
  "},"
    "{"
  "'name':'cutmax',"
  "'label':'Max Cut Time',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'70','l':'Sporty (70 ms)'},"
  "{'v':'100','l':'Normal (100 ms)'},"
  "{'v':'150','l':'Slow (150 ms)'}],"
  "'default':'100'"
  "},"
  "{"
  "'name':'minrpm',"
  "'label':'Min RPM',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':20000,"
  "'default':'2500'"
  "},"
  "{"
  "'name':'maxrpm',"
  "'label':'Max RPM',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1000,'max':20000,"
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
  "'default':'950'"
  "},"
  "{"
  "'name':'cuthysteresis',"
  "'label':'Cut Hysteresis (0-4095)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':5000,"
  "'default':'300'"
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
  "'label':'2-Step RPM (0=OFF)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':20000,"
  "'default':'0'"
  "},"
  "{"
  "'name':'twostepcut',"
  "'label':'2-Step Cut (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':1000,"
  "'default':'40'"
  "},"
  "{"
  "'name':'beta',"
  "'label':'Beta Features',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
  "},"
  "{"
  "'name':'testmode',"
  "'label':'Test functionality',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
  "}"
  "]";

AsyncWebServer server(80);
AsyncWebConfig conf;

struct cfgOptions
{
  char ssid[32];
  char password[32];

  bool enable;

  int trigMode;      // 0 piezo, 1 piezo inv, 2 hall, 3 hall inv, 4 loadcell
  int enableMode;    // 0 static, 1 hall, 2 hall inv

  int cutTime[5];    // ms
  int cutSmooth;     // 0 very hard, 1 hard, 2 normal, 3 soft, 4 very soft
  int cutMax;        // 

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
  int twoStepCut;

  bool beta;
  bool testmode;
} cfg;



void transferWebconfToStruct(String results)
{
    cfg.enable = conf.getBool("enable");

    cfg.trigMode = conf.getString("trigmode").toInt();
    cfg.enableMode = conf.getString("enablemode").toInt();

    cfg.cutTime[0] = conf.getInt("cuttime_1");
    cfg.cutTime[1] = conf.getInt("cuttime_2");
    cfg.cutTime[2] = conf.getInt("cuttime_3");
    cfg.cutTime[3] = conf.getInt("cuttime_4");
    cfg.cutTime[4] = conf.getInt("cuttime_5");

    cfg.cutSmooth = conf.getString("cutsmooth").toInt();
    cfg.cutMax = conf.getString("cutmax").toInt();

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
    cfg.twoStepCut = conf.getInt("twostepcut");

    cfg.beta = conf.getBool("beta");
    cfg.testmode = conf.getBool("testmode");
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
    String broadcastString = String(pressureValue) + "," + String(gearboxValue) + "," + String(lastRPM) + "," + String(cfg.cutTime[currRpmRange]);

    webSocket.loop();
    webSocket.broadcastTXT(broadcastString);

    delay(50);
  }
}

// Interrupt Service Routine (ISR) for the timer
void IRAM_ATTR Timer0_ISR()
{
  if (cfg.testmode)
  {
    cycleCounter++;
    if (cycleCounter == 1)
    {
      greenLED = true;
      digitalWrite(GREEN_LED, greenLED);
      disable_ign();
    }
    else if (cycleCounter >= 10)
    {
      // Turn the LED OFF after 9 more cycles (total 5 cycles = 500ms)
      greenLED = false;
      digitalWrite(GREEN_LED, greenLED);
      enable_ign_1();
      enable_ign_2();
      cycleCounter = 0;  // Reset the cycle counter
    }
  }
  else if (!cfg.testmode && greenLED)
  {
    greenLED = false;
    enable_ign_1();
    enable_ign_2();
    cycleCounter = 0;
  }
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
  Timer0_Cfg = timerBegin(1000000); // Prescaler of 80 gives 1us resolution

  // Set the compare value based on the desired frequency
  int compareValue = 1000000 / (frequency); // *2 for toggling on both compare events

  // Set the timer to trigger interrupt at compare value
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);  // ISR on compare match
  timerAlarm(Timer0_Cfg, compareValue, true, 0);   // Set compare value and auto-reload
}



void disable_ign()
{
  digitalWrite(IGN_FET_1, LOW);
  digitalWrite(IGN_FET_2, LOW);
  //digitalWrite(GREEN_LED, LOW);
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
  digitalWrite(RED_LED, (redLED ? HIGH : LOW));
  redLED = !redLED;

  quickRPM = 60000000 / timerReadMicros(Timer1_Cfg) * cfg.rpmMult;

  if (average_count < 5)
  {
    average_count++;
    return;
  }

  lastRPM = ((5 * 60000000) / timerReadMicros(Timer1_Cfg)) * cfg.rpmMult;
  //lastRPM = 60000000 / timerReadMicros(Timer1_Cfg); //* cfg.rpmMult;
  timerWrite(Timer1_Cfg, 0);
  average_count = 0;
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

  Serial.begin(57600);

  // Interrupt routine for ingition pulse counting / RPM calculation
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(RPM_PIN, countPulse, RISING);

  // Timer 1 initialisieren (Timer 1, Prescaler = 80 -> 1µs Auflösung)
  Timer1_Cfg = timerBegin(1000000); // Verwende Timer 1 mit 1 µs Auflösung

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

  setupTimer0(10);

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

      case 1: // Piezo Inverted
        pressureValue = 4095 - analogRead(PIEZO_PIN);
        break;

      case 2: // Hall
        pressureValue = analogRead(HALL2_PIN);
        break;

      case 3: // Hall Inverted
        pressureValue = 4095 - analogRead(HALL2_PIN);
        break;

      case 4: // Loadcell
        pressureValue = 0;
        break;
    }

    // Only read ADC if Hall is selected as enable mode
    if (cfg.enableMode == 1 || cfg.enableMode == 2)
      gearboxValue = analogRead(HALL1_PIN);
    else
      gearboxValue = 0;

    lastMillis = currMillis;
  }

  // Skip all quickshifter features, only 2-step remains
  if (cfg.enable)
  {
    // If no new RPM value since 500ms set RPM to 0
    if (timerReadMicros(Timer1_Cfg) > 500000)
      lastRPM = 0;

    // pressure sensor value went below hysteresis, re-enable triggering
    if (pressureValue < (cfg.cutSensitivity - cfg.cutHysteresis))
      waitHyst = false;

    // RPM is below minRPM, shift lever needs to be released before an upshift is allowed when RPM climbs above minRPM
    if (lastRPM < cfg.minRPM)
      waitHyst = true;

    /* ######################### Shift pressure detection and ignition cut ######################### */
    // Disable ignition when pressure is sensed, sufficient RPM, deadTime passed and pressure was low before (hysteresis prevents continuous triggers when holding shift lever up in top gear)
    if (lastRPM >= cfg.minRPM && coilsDisabled == 0 && waitHyst == false && pressureValue >= cfg.cutSensitivity && currMillis >= (lastCutMillis + cfg.deadTime))
    {
      disable_ign();
      coilsDisabled = 2;
      waitHyst = true;
  
      currRpmRange = map(lastRPM, cfg.minRPM, cfg.maxRPM, 0, 4);
  
      lastCutMillis = currMillis;
    }

  
    // Enable ignition 1 when dog rings touching or time passed
    if (coilsDisabled == 2)
    {
      if ((cfg.enableMode == 0 && currMillis >= (lastCutMillis + cfg.cutTime[currRpmRange])) || ((cfg.enableMode == 1 || cfg.enableMode == 2) && gearboxValue >= cfg.hallTouching))
      {
        enable_ign_1();
        coilsDisabled = 1;
      }
    }
  
    // Enable ignition 2 when gear fully engaged or time + smooth passed
    if (coilsDisabled == 1)
    {
      if ((cfg.enableMode == 0 && currMillis >= (lastCutMillis + cfg.cutTime[currRpmRange] + cfg.cutSmooth)) || ((cfg.enableMode == 1 || cfg.enableMode == 2) && gearboxValue >= cfg.hallEngaged))
      {
        enable_ign_2();
        coilsDisabled = 0;
      }
    }

    // BETA - Enable ignition if gear lever hasn't left center position (upshifting in highest gear, hall enable mode only)
    // maybe a bool variable (e.g. cancelingCut is needed to prevent coil 1 from getting enabled after 30ms and then coil 2 after the regular cuttime)
    if (cfg.beta && coilsDisabled == 2 && (cfg.enableMode == 1 || cfg.enableMode == 2) && (gearboxValue <= cfg.hallCenter + abs(cfg.hallTouching - cfg.hallCenter)/2) && currMillis >= (lastCutMillis + 30))
    {
      enable_ign_1();
      coilsDisabled = 1;
    }
    if (cfg.beta && coilsDisabled == 1 && (cfg.enableMode == 1 || cfg.enableMode == 2) && (gearboxValue <= cfg.hallCenter + abs(cfg.hallTouching - cfg.hallCenter)/2) && currMillis >= (lastCutMillis + 30 + cfg.cutSmooth))
    {
      enable_ign_2();
      coilsDisabled = 0;
    }
  
    //Safety feature to prevent engine shutting off in case of an issue
    if (coilsDisabled == 2 && currMillis >= (lastCutMillis + cfg.cutMax))
    {
      enable_ign_1();
      coilsDisabled = 1;
    }
    if (coilsDisabled == 1 && currMillis >= (lastCutMillis + cfg.cutMax + cfg.cutSmooth))
    {
      enable_ign_2();
      coilsDisabled = 0;
    }
  }

  /* ######################### Two Step Rev Limiter ######################### */
  // (just for fun, WIP)
  if (cfg.twoStepRPM > 0)
  {
    if (coilsDisabled == 0 && quickRPM > cfg.twoStepRPM)
    {
      disable_ign();
      coilsDisabled = 2;
      lastCutMillis = currMillis;
    }
    else if (coilsDisabled > 0 && currMillis >= (lastCutMillis + cfg.twoStepCut))
    {
      enable_ign_1();
      enable_ign_2();
      coilsDisabled = 0;
    }
  }
}
