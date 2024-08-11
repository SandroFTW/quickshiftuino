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
#define RED_LED 6
#define IGN_FET_1 1
#define IGN_FET_2 2
#define PIEZO_PIN 13
#define RPM_PIN 7

// GLOBAL VARIABLES
static unsigned long lastPulseMillis = 0; // Time of last RPM calculation
static unsigned long lastCutMillis = 0;   // Time of last ignition cut begin
static unsigned long currMillis = 0;      // millis()
static unsigned long lastMillis = 0;      // Last cycle millis() (currMillis - loopTime)
static unsigned int coilsDisabled = 0;    // Is ignition currently disabled?
static volatile int pulseCount = 0;       // Ignition interrupt count since last RPM calculation
static int currRpmRange = 0;              // 0-4 to choose cutoffTime from array
static float lastRPM = 0;                 // Last calculated RPM
static volatile bool redled = false;      // Used to toggle red LED on RPM pulse

static int sensorValue = 0;

String params = "["
  "{"
  "'name':'password',"
  "'label':'WiFi password',"
  "'type':"+String(INPUTPASSWORD)+","
  "'default':''"
  "},"
  // "{"
  // "'name':'opmode',"
  // "'label':'Operation Mode',"
  // "'type':"+String(INPUTSELECT)+","
  // "'options':["
  // "{'v':'PS','l':'Piezo Static'},"
  // "{'v':'PH','l':'Piezo Hall'},"
  // "{'v':'LS','l':'Loadcell Static'},"
  // "{'v':'LH','l':'Loadcell Hall'}],"
  // "'default':'PS'"
  // "},"
  "{"
  "'name':'cuttime1_1',"
  "'label':'Cuttime 1 (2500-4500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'62'"
  "},"
  "{"
  "'name':'cuttime1_2',"
  "'label':'Cuttime 1 (4500-6500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'58'"
  "},"
  "{"
  "'name':'cuttime1_3',"
  "'label':'Cuttime 1 (6500-8500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'52'"
  "},"
  "{"
  "'name':'cuttime1_4',"
  "'label':'Cuttime 1 (8500-10500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'50'"
  "},"
  "{"
  "'name':'cuttime1_5',"
  "'label':'Cuttime 1 (10500-12500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'50'"
  "},"
  "{"
  "'name':'cuttime2_1',"
  "'label':'Cuttime 2 (2500-4500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'66'"
  "},"
  "{"
  "'name':'cuttime2_2',"
  "'label':'Cuttime 2 (4500-6500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'62'"
  "},"
  "{"
  "'name':'cuttime2_3',"
  "'label':'Cuttime 2 (6500-8500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'54'"
  "},"
  "{"
  "'name':'cuttime2_4',"
  "'label':'Cuttime 2 (8500-10500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'54'"
  "},"
  "{"
  "'name':'cuttime2_5',"
  "'label':'Cuttime 2 (10500-12500)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':20,'max':200,"
  "'default':'52'"
  "},"
  "{"
  "'name':'minrpm',"
  "'label':'Min RPM',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1000,'max':20000,"
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
  "'label':'Dead time',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':1000,"
  "'default':'350'"
  "},"
  "{"
  "'name':'cutsensitivity',"
  "'label':'Cut sensitivity',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':2000,"
  "'default':'250'"
  "}"
  "]";

AsyncWebServer server(80);
AsyncWebConfig conf;

struct cfgOptions
{
  char ssid[32];
  char password[32];

  char opmode[4];      // PS, PH, LS, LH

  int cutTime1[5]; // ms
  int cutTime2[5]; // ms

  int minRPM;          // 1/min
  int maxRPM;          // 1/min

  float rpmMult;       // factor
  int deadTime;        // ms

  int cutSensitivity;
} cfg;



void transferWebconfToStruct(String results)
{
    //cfg.opmode = conf.getString("opmode");

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
    String broadcastString = String(sensorValue) + "," + String(lastRPM);

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
  digitalWrite(RED_LED, (redled ? HIGH : LOW));
  redled = !redled;
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
    sensorValue = (analogRead(PIEZO_PIN) - 2048) * (-1);

    lastMillis = currMillis;
  }

  /* ######################### Shift pressure detection and ignition cut ######################### */
  // Disable ignition when pressure is sensed and sufficient RPM
  if (lastRPM >= cfg.minRPM && coilsDisabled == 0 && sensorValue >= cfg.cutSensitivity && currMillis >= (lastCutMillis + cfg.deadTime))
  {
      disable_ign();
      coilsDisabled = 2;

      currRpmRange = map(lastRPM, cfg.minRPM, cfg.maxRPM, 0, 4);

      lastCutMillis = currMillis;
  }

  // Enable ignition 1 when dog rings touching
  //if (coilsDisabled == 2 && currMillis >= (lastCutMillis + 50))
  if (coilsDisabled == 2 && currMillis >= (lastCutMillis + cfg.cutTime1[currRpmRange]))
  {
    enable_ign_1();
    coilsDisabled = 1;
  }

  // Enable ignition 2 when gear fully engaged
  //if (coilsDisabled == 1 && currMillis >= (lastCutMillis + 55))
  if (coilsDisabled == 1 && currMillis >= (lastCutMillis + cfg.cutTime2[currRpmRange]))
  {
    enable_ign_2();
    coilsDisabled = 0;
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
