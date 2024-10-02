#include <WebSocketsServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebConfig.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <WiFi.h>

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
static volatile bool redLED = false;      // Used to toggle red LED on RPM pulse
static unsigned int coilsDisabled = 0;    // Is ignition currently disabled?
static volatile int pulseCount = 0;       // Ignition interrupt count since last RPM calculation
static int currRpmRange = 0;              // 0-4 to choose cutoffTime from array
static float lastRPM = 0;                 // Last calculated RPM

static float pressureValue = 0;
static float gearboxValue = 0;

String params = "["
  "{"
  "'name':'password',"
  "'label':'WiFi password',"
  "'type':"+String(INPUTPASSWORD)+","
  "'default':''"
  "},"
  "{"
  "'name':'enable',"
  "'label':'Enable Quickshiftuino',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'1'"
  "},"
  "{"
  "'name':'trigmode',"
  "'label':'Trigger Mode',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'piezo','l':'Piezo'},"
  "{'v':'hall','l':'Hall'},"
  "{'v':'loadcell','l':'Loadcell'}],"
  "'default':'hall'"
  "},"
  "{"
  "'name':'enablemode',"
  "'label':'Enable Mode',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'static','l':'Static (Time)'},"
  "{'v':'hall','l':'Hall'}],"
  "'default':'hall'"
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
  "'label':'Cut sensitivity (0-100)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':2000,"
  "'default':'250'"
  "},"
  "{"
  "'name':'halltouching',"
  "'label':'Hall dogs touching (0-100)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':20000,"
  "'default':'2000'"
  "},"
  "{"
  "'name':'hallengaged',"
  "'label':'Hall gear engaged (0-100)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':20000,"
  "'default':'3000'"
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

  char trigMode[3];      // piezo, hall, loadcell
  char enableMode[2];    // static, hall

  bool invertPressure;
  bool invertGearbox;

  int cutTime1[5]; // ms
  int cutTime2[5]; // ms

  int minRPM;          // 1/min
  int maxRPM;          // 1/min

  float rpmMult;       // factor
  int deadTime;        // ms

  int cutSensitivity;
  int hallTouching;
  int hallEngaged;

  int twoStepRPM;
  int twoStepHyst;
} cfg;



void transferWebconfToStruct(String results)
{
    cfg.enable = conf.getBool("enable");

    cfg.trigMode = conf.getString("trigmode");
    cfg.enableMode = conf.getString("enablemode");

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
    String broadcastString = String(sensorValue) + "," + String(lastRPM) + "," + String(hallValue);

    webSocket.loop();
    webSocket.broadcastTXT(broadcastString);

    delay(50);
  }
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
  digitalWrite(GREEN_LED, HIGH);
}

void countPulse()
{
  pulseCount++;
  digitalWrite(RED_LED, (redLED ? HIGH : LOW));
  redLED = !redLED;
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

  xTaskCreatePinnedToCore(push_debug, "CPU_0", 10000, NULL, 1, &Task1, 0);
}

void loop()
{
  currMillis = millis();

  if (currMillis >= (lastMillis + loopTime))
  {
    switch (cfg.trigMode)
    {
      case "piezo":
        pressureValue = analogRead(PIEZO_PIN) / 40.95f;
        break;

      case "hall":
        pressureValue = analogRead(HALL2_PIN) / 40.95f;
        break;

      case "loadcell":
        pressureValue = 0.f;
        break;
    }

    gearboxValue = analogRead(HALL1_PIN) / 40.95f;

    if (cfg.invertPressure) pressureValue = 100.f - pressureValue;
    if (cfg.invertGearbox) gearboxValue = 100.f - gearboxValue;

    lastMillis = currMillis;
  }

  /* ######################### Shift pressure detection and ignition cut ######################### */
  // Disable ignition when pressure is sensed and sufficient RPM
  if (lastRPM >= cfg.minRPM && coilsDisabled == 0 && pressureValue >= cfg.cutSensitivity && currMillis >= (lastCutMillis + cfg.deadTime))
  {
      disable_ign();
      coilsDisabled = 2;

      currRpmRange = map(lastRPM, cfg.minRPM, cfg.maxRPM, 0, 4);

      lastCutMillis = currMillis;
  }

  // Enable ignition 1 when dog rings touching or time 1 passed
  if (coilsDisabled == 2)
  {
    if ((cfg.enableMode == "static" && currMillis >= (lastCutMillis + cfg.cutTime1[currRpmRange])) ||
        (cfg.enableMode == "hall" && hallValue >= cfg.hallTouching))
    {
      enable_ign_1();
      coilsDisabled = 1;
    }
  }

  // Enable ignition 2 when gear fully engaged or time 2 passed
  if (coilsDisabled == 1)
  {
    if ((cfg.enableMode == "static" && currMillis >= (lastCutMillis + cfg.cutTime2[currRpmRange])) ||
        (cfg.enableMode == "hall" && hallValue >= cfg.hallEngaged))
    {
      enable_ign_2();
      coilsDisabled = 0;
    }
  }

  //Safety feature to prevent engine shutting off in case of an issue
  if (coilsDisabled == 2 && currMillis >= (lastCutMillis + 150))
  {
    enable_ign_1();
    coilsDisabled = 1;
  }
  if (coilsDisabled == 1 && currMillis >= (lastCutMillis + 160))
  {
    enable_ign_2();
    coilsDisabled = 0;
  }


  /* ######################### Two Step Rev Limiter ######################### */
  // (just for fun, WIP)
  if (cfg.twoStepRPM > 0)
  {
    if (lastRPM > cfg.twoStepRPM)
    {
      disable_ign();
      coilsDisabled = 2;
    }
    else if (lastRPM < (cfg.twoStepRPM - cfg.twoStepHyst))
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