#define DISABLE_DIAGNOSTIC_OUTPUT
#include <AccelStepper.h>
#include "OneButton.h"
#include <EEPROM.h>
#include <time.h>
#include <WiFi.h>
#include "BLESmartConfig.h"
#include "a4l_secrets.h"
#include "a4lUtilities.h"
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <FreeMonoBold7pt7b.h>
#include <FreeSansBold12pt7b.h>
#include <FreeSansBold7pt7b.h>
#include <FreeSansBold9pt7b.h>
#include <FreeSansBold5pt7b.h>
#include <FreeSansBold6pt7b.h>
#include <not_used_24.h>
#include <graphic_bitmaps.h>
#include <tick_24.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "esp_sleep.h"

char uid[20] = "";
           
DeviceInfo deviceInfo = {
  "Smart Pill Box",       //Device type
  "PB.1.1.0",             //Device version
  "ESP32",                //Device processor
  "SWITCH",               //Device category
  "Medication reminder system",//Device description
  "apps4leisure",         //Device manufacturer
  "PB",   
  10  
};

enum DoseMode {AM_ONLY, AM_PM};

struct MyObject{
  int8_t pillMatrix[2][7];
  char ssid[20];
  char password[20];
  DoseMode doseMode;
  int startYear;
  int startMonth;
  int startDay;
  int timeOffsetMins;
  unsigned long clientId;
  bool callWifiManager;   
  char initialised[4];
  char endbite[1];
} customVar;

#define RST_PIN 14
#define DC_PIN 27
#define CS_PIN 26
#define BUSY_PIN 32

#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DRIVER_CLASS GxEPD2_150_BN // DEPG0150BN 200x200, SSD1681, (FPC8101), TTGO T5 V2.4.1

#define MAX_DISPLAY_BUFFER_SIZE 65536ul
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
// adapt the constructor parameters to your wiring
//GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS(/*CS=*/CS_PIN, /*DC=*/DC_PIN, /*RST=*/RST_PIN, /*BUSY=*/BUSY_PIN));
GxEPD2_BW<GxEPD2_213_BN, GxEPD2_213_BN::HEIGHT> display(GxEPD2_213_BN(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RST_PIN, /*BUSY=*/ BUSY_PIN)); // DEPG0213BN 122x250, SSD1680
bool displayState = true;

// Define the dimensions of the display
const int MAIN_TOP = 44;

// Create a buffer to store the display content
//RTC_DATA_ATTR uint8_t displayBuffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8]; // 1 bit per pixel

String dayName[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

const uint8_t AM_START_H = 7;
const uint8_t AM_START_M = 0;
const uint8_t AM_END_H = 11;
const uint8_t AM_END_M = 35;

const uint8_t PM_START_H = 19;
const uint8_t PM_START_M = 0;
const uint8_t PM_END_H = 20;
const uint8_t PM_END_M = 43;

struct VoltageToPercent {
  float voltage;
  int percent;
};

const VoltageToPercent socTable[] = {
  {4.20, 100}, {4.15, 95}, {4.10, 90}, {4.05, 85},
  {4.00, 80},  {3.95, 75}, {3.90, 70}, {3.85, 65},
  {3.80, 60},  {3.75, 55}, {3.70, 50}, {3.65, 45},
  {3.60, 40},  {3.55, 30}, {3.50, 20}, {3.45, 15},
  {3.40, 10},  {3.30, 5},  {3.20, 0}
};

//#define uS_TO_S_FACTOR 1000000ULL
#define INACTIVITY_TIMEOUT 15000  // 20 seconds
unsigned long lastActivity = 0;

RTC_DATA_ATTR bool initalisePillMatrix = true;
RTC_DATA_ATTR bool pending_state = false;
RTC_DATA_ATTR int pending_period = 0;
RTC_DATA_ATTR int pending_day = 0;

typedef struct {
  uint16_t start;
  uint16_t end;
}DoseWindow;

struct DayPeriod {
  int day;   // 0–6 (Sun–Sat)
  int period;  // 0 or 1
  int segmentIndex;
  bool reload;
};

struct tm timeinfo;
struct DayPeriod today;

DoseWindow am_window;
DoseWindow pm_window;
DoseWindow midnight_window;

enum DoseResult {
  TODAY_AM,
  TODAY_PM,
  TOMORROW_AM
};

DoseMode doseMode = AM_PM;

int currentDay = -1;

bool matrixChanged = false;

const int IN1 = 4;
const int IN2 = 16;
const int IN3 = 17;
const int IN4 = 13;

const int HOME_PIN = 36;
const int DISPENSE_A_PIN = 25;
const int DISPENSE_B_PIN = 2;
const int CUP_PIN = 35;
const int LID_PIN = 34;
const int BATTERY_REF_PIN = 39;
const int DISPLAY_POWER_PIN = 21;
const int BUZZER_PIN = 22;
const int RESET_PIN = 0;
const int STEPPER_ENABLE_PIN = 15;
const int HALL_ENABLE_PIN = 12;

uint64_t wakeMask =
  (1ULL << DISPENSE_A_PIN) | 
 //(1ULL << DISPENSE_B_PIN) | 
 // (1ULL << LID_PIN) |
  (1ULL << CUP_PIN);

struct wake {
   byte TIMER = 1;
   byte EXT = 2;
   byte BOOT = 3;
}wakeReason;
int wake;

enum WiFiState {
  WIFI_IDLE,
  WIFI_CONNECTING,
  WIFI_CONNECTED,
  WIFI_FAILED,
  WIFI_DISCONNECTED,
  WIFI_SLEEP
};

WiFiState wifiState = WIFI_IDLE;
WiFiState lastWiFiState = WIFI_IDLE;
uint32_t wifiStartTime = 0;
const uint32_t WIFI_TIMEOUT = 50000;
//bool needsWiFi = false;

// Geometry
const int STEPS_PER_REV = 2055;//3010;   // 28BYJ‑48 gearbox
const int SEGMENTS = 15;
const int STEPS_PER_SEG = STEPS_PER_REV / SEGMENTS;

// Stepper object
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

OneButton resetButton(RESET_PIN, false);
OneButton dispense_A_Button(DISPENSE_A_PIN, false);
OneButton dispense_B_Button(DISPENSE_B_PIN, false);

// State machine
enum State { 
  PB_IDLE, 
  PB_HOME,
  PB_DISPENSE, 
  PB_MISSED_DOSE,
  PB_CONFIG_WIFI,
  PB_ADD_DEVICE,
  PB_CONNECT_WIFI,
  PB_MODE_CONFIG,
  PB_ALIGN_CAROUSEL,
};

State state = PB_IDLE;
State lastState;

enum DoseState : uint8_t {
  DOSE_NONE = 0,     // nothing scheduled / unused
  DOSE_TAKEN = 1,  // scheduled but not yet due
  DOSE_MISSED = 2,   // missed
  DOSE_ACTIVE = 3,   // current / highlight
};

enum timerWakeEvents {
  TMR_AM_MISSED,
  TMR_PM_MISSED,
  TMR_CROSS_DAYS,
  TM_NONE,
};

bool lastCupState = true;
bool lastLidState = true;
bool boot = true;


int day = 0;
int row = 1;

bool reload = false;


void printCenter(const String &text, int y, int offset = 0);

RTC_DATA_ATTR uint8_t currentPeriod = 0;
RTC_DATA_ATTR bool midnightWake = false;
RTC_DATA_ATTR int segmentIndex = 0;

RTC_DATA_ATTR bool simMode = false;   // Set to true to enable time simulation mode (for testing without waiting for real time to pass)
RTC_DATA_ATTR time_t simEpoch = 0;

bool wakeByCup = false;
bool wakeByDispenseButton = false;
bool warningBeep = false;
RTC_DATA_ATTR int wakeByPeriod = 0;

bool initSystem = false;

BLESmartConfig bleConfig;

unsigned long lastBeep = 0;

void setup() {

  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Disable brownout detector
  
  pinMode(CUP_PIN, INPUT_PULLUP);
  pinMode(LID_PIN, INPUT_PULLUP);
  pinMode(HOME_PIN, INPUT_PULLUP);
  pinMode(DISPENSE_A_PIN, INPUT_PULLUP);
  pinMode(DISPENSE_B_PIN, INPUT_PULLUP);

  //pinMode(DISPLAY_POWER_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Power on the display

   delay(100);

  peripheral_on();
 

  Serial.begin(115200);
  delay(500);

  uint64_t wakeupPins = esp_sleep_get_ext1_wakeup_status();
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  Serial.print("EXT1 wake status: 0x");
  Serial.println(wakeupPins, HEX);
  
  if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Dispense A triggered");
    wakeByDispenseButton = true;
  }

  /*
  if (wakeupPins & (1ULL << DISPENSE_A_PIN)) {
    Serial.println("Dispense A triggered");
    dispenseA();
    // DISPENSE A triggered
  }
*/

  if (wakeupPins & (1ULL << DISPENSE_B_PIN)) {
    Serial.println("Dispense B triggered");
      // DISPENSE B triggered
  }

  if (wakeupPins & (1ULL << CUP_PIN)) {
    Serial.println("Cup removed");
    wakeByCup = true;
      // DISPENSE B triggered
  }

  resetButton.attachClick(resetClick);
  dispense_A_Button.attachClick(dispenseA);
  //dispense_A_Button.attachDoubleClick(home);
  //dispense_A_Button.attachLongPressStart(reverse);
  dispense_B_Button.attachClick(dispenseB);

  String tmp_uid = WiFi.macAddress();
  tmp_uid.replace(":", "");      // Remove colons
  tmp_uid = "a4l-" + tmp_uid;    // Prepend "a4l-"
  strcat(uid, tmp_uid.c_str());
  Serial.println(uid);
 
 

  EEPROM.begin(512);
  EEPROM.get(0, customVar);

  // Initialize last states which are used to detect changes in the loop and trigger display updates
  // We invert the state here because the loop will detect a change and update the display accordingly on boot
  
  doseMode = customVar.doseMode;


  customVar.timeOffsetMins = 0;
 // customVar.pillMatrix[1][5] = DOSE_TAKEN; //cliff
  am_window.start = AM_START_H * 60 + AM_START_M;
  am_window.end = AM_END_H * 60 + AM_END_M + customVar.timeOffsetMins;
  pm_window.start = PM_START_H * 60 + PM_START_M;
  pm_window.end = PM_END_H * 60 + PM_END_M + customVar.timeOffsetMins;
  //24 Hours plus 3 hours to take it way past any DST adjustments
  midnight_window.end = 27 * 60; 

  doseMode = AM_PM;  //cliff
  simMode = false;

   strcpy(customVar.initialised,"YES");

  
  getWakeReason();
 
  //Serial.println(customVar.callWifiManager);
  //Serial.println(wake);
  //Serial.println(wakeReason.BOOT);
  //customVar.callWifiManager = true;
  if(wake == wakeReason.BOOT){
   
    initalisePillMatrix = true;
    if(customVar.callWifiManager){
      initPillMatrix();
      state = PB_CONFIG_WIFI;
      return;
    }else{

      wifiRequest();
      wifiSetRTC();
      getTimeWithRetry();

      initPillMatrix();
      display.init(0, true, 2, false); 
      display.setRotation(3);               // Set the rotation (0, 1, 2, or 3) depending on your display
      display.fillScreen(GxEPD_WHITE);      // Fill the screen with white background
      buildMainDisplay();
      
    }
    
  }else{
  
    display.init(0, false); 
    display.setRotation(3);
    display.firstPage();
    do{
      display.setPartialWindow(0, 0, 250, 122);
      messageHeader();
      display.setFont(&FreeSansBold7pt);
      display.setTextColor(GxEPD_BLACK);
      drawMatrix();

      display.drawRect(175, MAIN_TOP + 51, 2, 48, GxEPD_BLACK);
      display.drawRect(62, MAIN_TOP + 50, 2, 48, GxEPD_BLACK);
      drawState("IDLE");

    } while (display.nextPage());
    display.firstPage();
  }
  //Initalise the pinmMode after the display init!!
  
  getTimeWithRetry();

  if(simMode) advanceToNextPeriod();
  Serial.println(timeinfo.tm_year + 1900);
  today = getDayPeriod(timeinfo);
  currentDay = today.day;
  Serial.println("Initialising timers and windows...");
  Serial.println("Today is day " + String(today.day) + " period " + String(today.period));
  lastCupState = getCupState();
  lastLidState = getLidState();

  
  simMode = false;
  ///if(!getTimeWithRetry()) wifiRequest();
  //simMode = true;
  //if(wake == wakeReason.TIMER && simMode)simulateCycleDeepSleep();

  initaliseTimers();
  Serial.println("Week day");
  Serial.println(timeinfo.tm_wday);
  if(simMode){
    esp_sleep_enable_timer_wakeup(2ULL * 1000000ULL);
    esp_deep_sleep_start();
  }

 
}

void advanceToNextPeriod() {

  struct tm simulatedTime = timeinfo;

  switch (wakeByPeriod) {
    // After AM processing:
    // next wake should trigger PM expiry
    case 0:

      simulatedTime.tm_hour = AM_END_H;
      simulatedTime.tm_min  = AM_END_M - 1;
      simulatedTime.tm_sec  = 0;
      break;

    // After PM processing:
    // move to next day just before AM expiry
    case 1:
      if (doseMode == AM_ONLY){
        break;
      }
      simulatedTime.tm_hour = PM_END_H;
      simulatedTime.tm_min  = PM_END_M - 1;
      simulatedTime.tm_sec  = 0;
      break;

    // Midnight rollover:
    // also position just before AM expiry
    case 3:
      simulatedTime.tm_mday += 1;
      simulatedTime.tm_hour = AM_END_H;
      simulatedTime.tm_min  = AM_END_M - 1;
      simulatedTime.tm_sec  = 0;
      break;
  }

  // Normalize date/time
  mktime(&simulatedTime);

  timeinfo = simulatedTime;

  setRTCfromTimeinfo(&timeinfo);
  
  Serial.print("Simulated time set to: ");
  Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
}

void setRTCfromTimeinfo(struct tm *tminfo) {
  time_t t = mktime(tminfo);
  struct timeval now = {
    .tv_sec = t,
    .tv_usec = 0
  };
  settimeofday(&now, nullptr);
}

void loop() {
 // digitalWrite(STEPPER_ENABLE_PIN, HIGH); // Power off the display`
 
  bool beepPass = false;
  
  dispense_A_Button.tick();
  dispense_B_Button.tick();
  resetButton.tick();

 // wifiLoop();

  bool currentCupState = getCupState();
  bool currentLidState = getLidState();
  bool currentHomeState = getHomeState();


  currentLidState = 1;
 // currentCupState = 1;
 // currentHomeState = 1;
  Serial.print("Cup: "); Serial.print(currentCupState);
  Serial.print(" Lid: "); Serial.print(currentLidState);
  Serial.print(" Home: "); Serial.println(currentHomeState); 
    
  unsigned long currentMillis = millis();
   //goToSleep();
  //Serial.println(currentMillis - lastActivity);
  if (currentMillis - lastActivity > INACTIVITY_TIMEOUT){// && state == PB_IDLE) {  // && currentLidState put this back
      Serial.println(currentMillis - lastActivity);
      Serial.println("Sleep..");
    goToSleep();
  }

  //If the cup is in and the lid is closed we dont need a beep
  //if(currentCupState && currentLidState) warningBeep = false;

  //Beep with double beep if the cup is in but the carousel has dispensed, even if the lid is open
  //If nothing to dispense but the cup is out or lid open give a single beep
  if(pending_state || warningBeep){
    
    if (currentMillis - lastBeep >= 10000) {
      if(pending_state){
        Serial.println("Beep pending");
        beepPattern(2);
      }
      if(warningBeep && !warningBeep) beepPattern(1);
      lastBeep = currentMillis;
    }
  }

  //The dispense button will not fire if the CPU is in deep sleep so we detect the wake reason
  //and trigger the dispense function
  if(wakeByDispenseButton){
    wakeByDispenseButton = false;
    dispenseA();
  }

  //If a dose has been dispensed, its still in the cup and a warning set, until the cup is removed
  //detect the cup removal and then confirm by updating the matrix status and cancel warning
  if (currentCupState != lastCupState || wakeByCup) {
    lastCupState = currentCupState;
    wakeByCup = false;
    if(pending_state){
      confirmDoseTaken();
    }else{
      warningBeep = true;
    }
  } 

  if (currentLidState != lastLidState) {
    lastLidState = currentLidState;
    if(currentLidState){
      Serial.println("Resync");
      state = PB_HOME;
    }
    warningBeep = true;
  } 

  switch (state) {

    case PB_CONFIG_WIFI:
      if (lastState != state){
        lastState = state;
        setBLESmartConfig();
      }
      break;

    case PB_MODE_CONFIG:
      if (lastState != state){
        lastState = state;
        display.firstPage();
        do{
          display.setPartialWindow(0, MAIN_TOP - 5, display.width(), 85);
            dosesSetupMessage();
        } while (display.nextPage());
        display.firstPage();
      }
      break;
       
    case PB_ADD_DEVICE:
      if (lastState != state){
        state = lastState;
        bleConfig.stopBLESmartConfig();
        Serial.println("Add device");
        registerDevice();
      }
      break;

    case PB_HOME:
      if (lastState != state){
        lastState = state;
    
        moveToHome();
      }
      break;

    case PB_DISPENSE:
      if (lastState != state){
        lastState = state;
        moveCarrouselTo(1);
        delay(500);
        pending_state = true;
        state = PB_IDLE;
      }
      break;

    case PB_ALIGN_CAROUSEL:
      if (lastState != state){
        stateRefresh("ALIGNING");
        lastState = state;
        moveCarrouselTo(segmentIndex);
      }
      state = PB_IDLE;
      break;
      
    case PB_IDLE:
      if (lastState != state){
        stateRefresh("IDLE");
        lastActivity = millis();
        lastState = state;
      }
      break;
  }

  delay(5);
}

void peripheral_off(){
  pinMode(DISPLAY_POWER_PIN, INPUT);
  digitalWrite(DISPLAY_POWER_PIN, HIGH); // Power off the display`
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // Power off the display`
  digitalWrite(HALL_ENABLE_PIN, LOW);
}

void peripheral_on(){
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, LOW); // Power off the display`
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, LOW); // Power off the display`
  pinMode(HALL_ENABLE_PIN, OUTPUT);
  digitalWrite(HALL_ENABLE_PIN, HIGH); // Power off the display`
}

bool getCupState(){
  return !digitalRead(CUP_PIN);
}
bool getLidState(){
  return !digitalRead(LID_PIN);
}
bool getHomeState(){
  return !digitalRead(HOME_PIN);
}

void moveCarrouselTo(int segment){
  stepper.setMaxSpeed(200);
  stepper.setAcceleration(200);
  stepper.move(-STEPS_PER_SEG * segment);
  while (stepper.distanceToGo() != 0) stepper.run();
  stepper.stop();
  stepper.disableOutputs();
}
void moveToHome(){
  
  stateRefresh("HOMEING");
 
  stepper.setMaxSpeed(200);
  stepper.setAcceleration(200);
  //Move the carousel until we reach the home hall device
  while (!getHomeState()) {
    stepper.setSpeed(-200);
    stepper.runSpeed();
  } 
  //Now allow an adjustment to align the carousel perfectly 
  //stepper.move(-23);
  //while (stepper.distanceToGo() != 0) stepper.run();

  //Stop the stepper motor and disable the drivers
  if(segmentIndex == 0){
    stepper.stop();
    stepper.disableOutputs();
    state = PB_IDLE;
  }else{
    Serial.println(segmentIndex);
    state = PB_ALIGN_CAROUSEL;
  }
}

void beepPattern(int beeps){
  for (int x = 0; x < beeps; x++){
    digitalWrite(BUZZER_PIN, HIGH);   // start beep
    delay(100);             // short beep only
    digitalWrite(BUZZER_PIN, LOW);    // stop beep
    delay(500);
  }
}

void confirmDoseTaken(){
  customVar.pillMatrix[pending_period][pending_day] = DOSE_TAKEN;
  pending_state = false;
  pending_period = 0;
  pending_day = 0;
  segmentIndex ++;
  getCarouselStatus();
  matrixRefresh();
}

bool waitForBusy(uint32_t timeoutMs = 3000)
{
    uint32_t start = millis();
    while (digitalRead(BUSY_PIN) == HIGH)
    {
        if (millis() - start > timeoutMs)
            return false; // timed out
        delay(1);
    }
    return true;
}

void goToSleep(){
  //Disable wifi and change the icon to sleep
  if (WiFi.status() == WL_CONNECTED){
    wifiRefresh(WIFI_SLEEP);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  //Wait for the screen to finish updating
 if (!waitForBusy()) {
    // BUSY never cleared → panel probably lost power
    // handle gracefully instead of hanging forever
}
  //display.hibernate(); // optional but recommended
  delay(500);
  peripheral_off();
  delay(500);
  //digitalWrite(DISPLAY_POWER_PIN, LOW); // Power off the display
  Serial.println("Going to sleep...");
  // EXT0 for dispense button
  esp_sleep_enable_ext0_wakeup((gpio_num_t)DISPENSE_A_PIN, 1);
  //esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
  if(!pending_state){
    int sleep_until = nextWakeTime();
    esp_sleep_enable_timer_wakeup((uint64_t)sleep_until * 60ULL * 1000000ULL);
  }else{
    esp_sleep_enable_timer_wakeup((uint64_t)10 * 1000000ULL);
  }
 
  esp_deep_sleep_start();
}



#pragma region DISPENSE_HELPERS

void resetClick(){
  Serial.println("Resetting device...");
  //EEPROM.put(0, MyObject()); // Write a default-constructed MyObject to reset
  //EEPROM.commit();
  //delay(100);
  //ESP.restart();
}

void dispenseA() {
  
  //The main function of this button it to dispense a dose, calclate the dose to dispence
  int period = today.period;
  uint16_t am_time_left = amTimeLeft();
  uint16_t pm_time_left = pmTimeLeft();

  if(doseMode == AM_ONLY){
    //DayWeek today = getDayWeek(timeinfo);
    if(am_time_left > 0){
      //period = today.period;
      dispenseDose();
    }else{
      Serial.print("Dose not in Window!");
      return;
    }
  }

  if(doseMode == AM_PM){
    if ((period == 0 && am_time_left > 0) || (period == 1 && pm_time_left > 0)){
      dispenseDose();
    }else{
      Serial.print("Dose not in Window!");
      return;
    }
  }
 
}

void dispenseB() {
  Serial.println(state);
  if(state ==  PB_MODE_CONFIG){
    doseMode = AM_PM;
    state = PB_ADD_DEVICE;
    customVar.doseMode = doseMode;
    EEPROM.put(0, customVar);
    EEPROM.commit();
    return;
  }else{
    //Reload carousel if the segmentCounter = 0 we can move the carousel to 
    //the home position and clear the pillMatrix ready for a new cycle
    segmentIndex = 0;
    if(segmentIndex == 0){
      state = PB_HOME;
      initPillMatrix();
      matrixRefresh();
      //sendPillMatrix();
    }else{
      Serial.println("Carousel not ready to load");
    }
  }
 
  /*
  state = PB_HOME;
  Serial.println("Dispense B");
  */
}

bool dispenseDose(){

  if(!getCupState()){
    Serial.println("Cup not in dispencer!");
    return false;
  }

  if(!getLidState()){
  //  Serial.println("The lid is open!");
  //  return false;
  }

  //DayWeek tomorrow = getDayWeek(timeinfo, 1);
  bool todayIsUntouched = (customVar.pillMatrix[today.period][today.day] == DOSE_NONE || customVar.pillMatrix[today.period][today.day] == DOSE_ACTIVE);

  if(!todayIsUntouched){
    Serial.println("Dose already taken, ignoring button press");
    return false;
  }else{
    pending_period = today.period;
    pending_day = today.day;
    //customVar.pillMatrix[period][day] = DOSE_TAKEN;
    //segmentIndex ++;
    //getCarouselStatus();
    //matrixRefresh();
    state = PB_DISPENSE;

  }
  return true;
}

DayPeriod getCarouselStatus(){
  DayPeriod out;
  int day, period;
  int seg = 0;
  //int dayNum = timeinfo.tm_wday;
  bool reload = false;
  for (int i = 0; i < 14; i++) {
    if (doseMode == AM_PM) {
      day = i / 2;
      period = i % 2;
      //periodCounter = dayNum * 2;// + currentPeriod;  
    }else{  // AM_ONLY mode
      period = i / 7;   // 0 or 1
      day  = i % 7;   // 0–
    }
    int pillState = customVar.pillMatrix[period][day];
    if(pillState == DOSE_TAKEN) seg ++;
    if(pillState == DOSE_NONE || pillState == DOSE_ACTIVE){
      customVar.pillMatrix[period][day] = DOSE_ACTIVE;
      out.period = period;
      out.day = day;
      out.segmentIndex = seg;
      out.reload = false;
      return out;
    } 
  }
  out.day = 7;
  out.period = 1;
  out.segmentIndex = seg;
  out.reload = true;
  return out;
}

void initPillMatrix(){
  for (int p = 0; p < 2; p++){
    for (int d = 0; d < 7; d++){
      customVar.pillMatrix[p][d] = DOSE_NONE;
    }
  }
}
#pragma endregion DISPENSE_HELPERS

#pragma region WIFI_HELPERS
bool wifiRequest() {

  wifiRefresh(WIFI_CONNECTING);
  WiFi.mode(WIFI_STA);
  WiFi.begin(customVar.ssid,customVar.password);
 
  unsigned long startAttemptTime = millis();
  Serial.println("Requesting WiFi connection...");
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    Serial.print(".");
    delay(500);
  }
  if(WiFi.status() == WL_CONNECTED){
    wifiRefresh(WIFI_CONNECTED);
    Serial.println("WiFi connected");
 
    return true;
  }
  wifiRefresh(WIFI_FAILED);
  return false;
}

bool ensureWifi() {

  // Already connected
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  for (uint8_t i = 0; i < 3; i++) {

    Serial.println("Connecting to WiFi...");
    
    if (wifiRequest()) {

      // Give connection a moment
      uint32_t start = millis();

      while (WiFi.status() != WL_CONNECTED &&
             millis() - start < 10000) {
        delay(100);
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected");
        return true;
      }
    }

    Serial.println("WiFi retry failed");
    delay(2000);
  }

  Serial.println("WiFi unavailable");
  return false;
}

void wifiSetRTC(){

  configTime(0, 0, "time.google.com", "time.cloudflare.com");
  setenv("TZ", "GMT0BST,M3.5.0/1,M10.5.0/2", 1);
  tzset();

}

void handleJson(const String& jsonPayload) {

  Serial.println("Callback triggered with JSON payload:");
  Serial.println(jsonPayload);

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonPayload);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  // --------------------------------------------------------------------------
  // WIFI SSID
  // --------------------------------------------------------------------------
  if (!doc["wifi_ssid"].isNull()) {
    const char* ssid = doc["wifi_ssid"] | "";
    strncpy(customVar.ssid, ssid, sizeof(customVar.ssid) - 1);
    customVar.ssid[sizeof(customVar.ssid) - 1] = '\0';
    Serial.print("SSID: ");
    Serial.println(customVar.ssid);
  } else {
    Serial.println("Key 'wifi_ssid' missing");
  }
  // --------------------------------------------------------------------------
  // WIFI PASSWORD
  // --------------------------------------------------------------------------
  if (!doc["wifi_password"].isNull()) {
    const char* password = doc["wifi_password"] | "";
    strncpy(customVar.password, password, sizeof(customVar.password) - 1);
    customVar.password[sizeof(customVar.password) - 1] = '\0';
    Serial.print("Password: ");
    Serial.println(customVar.password);
  } else {
    Serial.println("Key 'wifi_password' missing");
  }
  // --------------------------------------------------------------------------
  // CLIENT ID
  // --------------------------------------------------------------------------
  if (!doc["clientId"].isNull()) {
    const char* tmpChar = doc["clientId"];
    customVar.clientId = atol(tmpChar);
    Serial.println(tmpChar);
  } else {
    Serial.println("Key 'clientId' is missing!");
  }
  
  // --------------------------------------------------------------------------
  // Save config
  // --------------------------------------------------------------------------
  customVar.callWifiManager = false;
  strcpy(customVar.initialised,"YES");

  getTimeWithRetry();

  struct tm setupDate = getLastSunday(timeinfo);
  customVar.startYear = setupDate.tm_year;
  customVar.startMonth = setupDate.tm_mon;
  customVar.startDay = setupDate.tm_mday;
 
  EEPROM.put(0, customVar);

  if (!EEPROM.commit()) {
    Serial.println("EEPROM commit failed");
    return;
  }

  Serial.println("Config saved");
  // --------------------------------------------------------------------------
  // Shutdown BLE Config
  // --------------------------------------------------------------------------
  bleConfig.stopBLESmartConfig();
  // --------------------------------------------------------------------------
  // Change state
  // --------------------------------------------------------------------------
  state = PB_MODE_CONFIG;
   // state = PB_ADD_DEVICE;
}

/*
void handleJson(const String& jsonPayload) {
  Serial.println("Callback triggered with JSON payload:");
  Serial.println(jsonPayload);
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonPayload);

  if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return; // Exit if deserialization failed
  }

  // Check and copy each key only if it exists
  if (!doc["wifi_ssid"].isNull()) {
      const char* tmpChar = doc["wifi_ssid"];
      strcpy(customVar.ssid, tmpChar);
      Serial.println(tmpChar);
  } else {
      Serial.println("Key 'wifi_ssid' is missing!");
  }

  if (!doc["wifi_password"].isNull()) {
      const char* tmpChar = doc["wifi_password"];
      strcpy(customVar.password, tmpChar);
      Serial.println(tmpChar);
  } else {
      Serial.println("Key 'wifi_password' is missing!");
  }

  if (!doc["clientId"].isNull()) {
      const char* tmpChar = doc["clientId"];
      customVar.clientId = atol(tmpChar);
      Serial.println(tmpChar);
  } else {
      Serial.println("Key 'clientId' is missing!");
  }
  
  bleConfig.stopBLESmartConfig();
  customVar.callWifiManager = false;
  EEPROM.put(0, customVar);
  EEPROM.commit();
  Serial.println("Config saved");
  state = PB_MODE_CONFIG;

}

*/

void setBLESmartConfig(){
  wifiSetupMessage();
  String deviceName = "a4l-";
  deviceName += deviceInfo.type_short;
  bleConfig.begin(deviceName.c_str());
  bleConfig.setServiceUUID("12345678-1234-5678-1234-56789abcdef0");
  bleConfig.setCharacteristicUUID("abcdef01-1234-5678-1234-56789abcdef1");
  bleConfig.setJsonCallback(handleJson);
}


bool registerDevice() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return false;
  }

  // Create JSON document
  JsonDocument doc;

  // Root fields
  doc["message_type"]   = "insert_update_device";
  doc["device_type"]    = deviceInfo.type;
  doc["position"]       = deviceInfo.position;
  doc["friendly_name"]  = "Smart Pillbox";
  doc["type_short"]     = deviceInfo.type_short;
  doc["device_id"]      = uid;
  doc["client_id"]      = customVar.clientId;
  doc["version"]        = deviceInfo.version;
  doc["ip_address"]     = WiFi.localIP().toString();
  doc["device_processor"] = deviceInfo.processor;
  // --------------------------------------------------------------------------
  // json_config
  // --------------------------------------------------------------------------
  JsonObject jsonConfig = doc["json_config"].to<JsonObject>();

  jsonConfig["offset"] = 0;
  jsonConfig["mode"]   = (doseMode == AM_ONLY) ? "AM_ONLY" : "AM_PM";
  // Times array
  JsonArray timesArray = jsonConfig["times"].to<JsonArray>();

  const char* times[] = {
    "07:00",
    "12:00",
    "19:00",
    "22:00"
  };

  for (size_t i = 0; i < 4; i++) {
    JsonObject timeObj = timesArray.add<JsonObject>();
    timeObj["time"] = times[i];
  }
  // --------------------------------------------------------------------------
  // json_settings
  // --------------------------------------------------------------------------
  JsonArray settingsArray = doc["json_settings"].to<JsonArray>();

  for (size_t i = 0; i < 7; i++) {
    JsonArray inner = settingsArray.add<JsonArray>();
    inner.add(0);
    inner.add(0);
  }
  // --------------------------------------------------------------------------
  // Serialize JSON
  // --------------------------------------------------------------------------
  String jsonString;
  serializeJson(doc, jsonString);

  Serial.println("Sending JSON:");
  Serial.println(jsonString);
  // --------------------------------------------------------------------------
  // Send to backend
  // --------------------------------------------------------------------------
  String response = sendToBackend(jsonString);
  if (response.length() == 0) {
    Serial.println("Empty backend response");
    return false;
  }

  Serial.println("Backend response:");
  Serial.println(response);
  // --------------------------------------------------------------------------
  // Parse response
  // --------------------------------------------------------------------------
  JsonDocument resp;
  DeserializationError error = deserializeJson(resp, response);

  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return false;
  }

  bool success = resp["success"] | false;
  const char* action = resp["action"] | "";

  
  // --------------------------------------------------------------------------
  // Update display
  // --------------------------------------------------------------------------
  display.firstPage();
  do {

    display.setPartialWindow(0, MAIN_TOP - 5, display.width(), 85);
    drawMatrix();
    display.drawRect(175, MAIN_TOP + 51, 2, 48, GxEPD_BLACK);
    display.drawRect(62, MAIN_TOP + 50, 2, 48, GxEPD_BLACK);
    drawState("IDLE");
  } while (display.nextPage());

  display.firstPage();
  state = PB_IDLE;

  return success;
}
/*
bool registerDevice() {

  if (WiFi.status() != WL_CONNECTED) return false;

  // Allocate enough memory for the full structure
  JsonDocument doc;

  // Root fields
  doc["message_type"] = "insert_update_device";
  doc["device_type"] = deviceInfo.type;
  doc["position"] = deviceInfo.position;
  doc["friendly_name"] = "Smart Pillbox";
  doc["type_short"] = deviceInfo.type_short;
  doc["device_id"] = uid;
  doc["client_id"] = customVar.clientId;
  doc["version"] = deviceInfo.version;
  doc["ip_address"] = WiFi.localIP().toString();
  doc["device_processor"] = deviceInfo.processor;

  // json_config object
  JsonObject jsonConfig = doc.createNestedObject("json_config");

  // times array
  JsonArray timesArray = jsonConfig.createNestedArray("times");
  const char* times[] = {"07:00", "12:00", "19:00", "22:00"};

  for (size_t i = 0; i < 4; ++i) {
    JsonObject timeObj = timesArray.createNestedObject();
    timeObj["time"] = times[i];
  }

  jsonConfig["offset"] = 0;
  jsonConfig["mode"] = (doseMode == AM_ONLY ? "AM_ONLY" : "AM_PM");

  // json_settings array
  JsonArray settingsArray = doc.createNestedArray("json_settings");

  for (size_t i = 0; i < 7; ++i) {
    JsonArray inner = settingsArray.createNestedArray();
    inner.add(0);
    inner.add(0);
  }

  // Serialize
  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);

  // Send to backend
  String response = sendToBackend(jsonString);

  // Parse response
  JsonDocument resp;
  DeserializationError error = deserializeJson(resp, response);

  if (error) {
    Serial.print("JSON Parsing failed: ");
    Serial.println(error.c_str());
    return false;
  }

  bool success = resp["success"];
  const char* action = resp["action"];

  Serial.print("Success: ");
  Serial.println(success ? "Yes" : "No");

  Serial.print("Action: ");
  Serial.println(action);

  // Update display
  display.firstPage();
  do {
    display.setPartialWindow(0, MAIN_TOP - 5, display.width(), 85);
    drawMatrix();
    display.drawRect(175, MAIN_TOP + 51, 2, 48, GxEPD_BLACK);
    display.drawRect(62, MAIN_TOP + 50, 2, 48, GxEPD_BLACK);
    drawState("IDLE");
  } while (display.nextPage());
  display.firstPage();

  state = PB_IDLE;
  return true;
}
*/

/*
bool registerDevice(){

  if(WiFi.status() != WL_CONNECTED) return false;

  JsonDocument doc;

  // Add root level properties
  doc["message_type"] = "insert_update_device";
  doc["device_type"] = deviceInfo.type;
  doc["position"] = deviceInfo.position;
  doc["friendly_name"] = "Smart Pillbox";
  doc["type_short"] = deviceInfo.type_short;
  doc["device_id"] = uid;
  doc["client_id"] = customVar.clientId;
  doc["version"] = deviceInfo.version;
  doc["ip_address"] = WiFi.localIP().toString(); // Get IP address as String
  doc["device_processor"] = deviceInfo.processor;

  // Create the "json_config" object
  JsonObject jsonConfig = doc["json_config"].to<JsonObject>();

  // Create the "times" array within "json_config"
  JsonArray timesArray = jsonConfig["times"].to<JsonArray>();

  // Add time objects to the "times" array

  const char* times[] = {"07:00", "12:00", "19:00", "22:00"};
  for (size_t i = 0; i < 4; ++i) {
    JsonObject timeObj = timesArray.add<JsonObject>();
    timeObj["time"] = times[i];
  }

  // Add the "offset" property to "json_config"
  jsonConfig["offset"] = 0;
  jsonConfig["mode"] = doseMode == AM_ONLY ? "AM_ONLY" : "AM_PM";

  // Create the "json_settings" array within the root 
  JsonArray settingsArray = doc["json_settings"].to<JsonArray>();
  for (size_t i = 0; i < 7; ++i) {
    JsonArray innerArray = settingsArray.add<JsonArray>();
    for (size_t j = 0; j < 2; ++j) {
      innerArray.add(0);
    }
  }

  // Serialize the JSON document to a String
  String jsonString;
  serializeJson(doc, jsonString);

  Serial.println(jsonString);
  // Send the JSON to your backend (use your preferred method: HTTP POST, MQTT, etc.)
  String response = sendToBackend(jsonString);
  // Parse the JSON response
  DeserializationError error = deserializeJson(doc, response);

  // Check for parsing errors
  if (error) {
      Serial.print("JSON Parsing failed: ");
      Serial.println(error.c_str());
      return false;
  }

  // Extract values
  bool success = doc["success"];  // Read success flag
  const char* action = doc["action"];  // Read action value

  // Output the results
  Serial.print("Success: ");
  Serial.println(success ? "Yes" : "No");
  
  Serial.print("Action: ");
  Serial.println(action);

  display.firstPage();
  do{
    display.setPartialWindow(0, MAIN_TOP - 5, display.width(), 85);
     
      drawMatrix();

      display.drawRect(175, MAIN_TOP + 51, 2, 48, GxEPD_BLACK);
      display.drawRect(62, MAIN_TOP + 50, 2, 48, GxEPD_BLACK);

      //drawCupStatus(getCupState());
      //drawLidStatus(getLidState());
      drawState("IDLE");

  } while (display.nextPage());
  display.firstPage();

  state = PB_IDLE;

  return true;
}
  */

String sendToBackend(const String &data) {

  HTTPClient http;

  http.begin("http://apps4leisure.com/smart_home/smart_home_endpoint.php");  // URL passed as an argument
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(data);
  String responseData = "";
  // If the request was successful, read the response
  if (httpResponseCode > 0) {
    responseData = http.getString();  // Get the response body
    Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    Serial.println("Response: " + responseData);  // Print the response data
  } else {
    Serial.printf("Error on sending %s: %s\n",  http.errorToString(httpResponseCode).c_str());
  }

  http.end();
  
  return responseData;  // Re

}

bool sendPillMatrix() {
 
  //We need wifi hear and it may already be connected, if so dont re-connect
  //if (WiFi.status() != WL_CONNECTED){
  //  if(!wifiRequest()) return false;
  //}
  if (!ensureWifi()) {
    return false;
  }

  JsonDocument doc;
  doc["message_type"] = "insert_update_device";
  doc["device_id"] = uid;
  doc["battery_level"] = 0;

  // Create json_settings as a nested array in *this* document
  JsonArray outer = doc["json_settings"].to<JsonArray>();
  //JsonArray outer = doc.createNestedArray("json_settings");

  for (int i = 0; i < 7; i++) {
    JsonArray inner = outer.add<JsonArray>();
    //  JsonArray inner = outer.createNestedArray();
    for (int j = 0; j < 2; j++) {
      inner.add(customVar.pillMatrix[j][i]);
    }
  }

  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);

  if (WiFi.status() == WL_CONNECTED) {
    sendToBackend(jsonString);
  } else {
    Serial.println("WiFi Disconnected");
  }

  return true;

  /*
  // Create a JSON document
  StaticJsonDocument<1024> doc;
  StaticJsonDocument<512> tempDoc;
  //JsonDocument doc;
 
  // Add data to the JSON document
  doc["message_type"] = "insert_update_device";
  doc["device_id"] = uid;
  //JsonObject data = doc["data"].to<JsonObject>();
  doc["battery_level"] = 0;//readBatteryPower();

  //JsonDocument tempDoc;
  JsonArray array = tempDoc.to<JsonArray>(); // Create the nested array structure

  for (int i = 0; i < 7; i++) {
    JsonArray nested = array.add<JsonArray>(); // Create inner arrays
    for (int j = 0; j < 2; j++) {
      nested.add(customVar.pillMatrix[j][i]); // Add boolean values
    }
  }
  
  doc["json_settings"] = array;

  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);

  // Send the JSON string to the backend
  if (WiFi.status() == WL_CONNECTED) {
    sendToBackend(jsonString);
  }else {
    Serial.println("WiFi Disconnected");
  }
  return true;
  */
}

#pragma endregion WIFI_HELPERS

#pragma region TIME_HELPERS

int nextWakeTime(){
  
  int sleep_until = 0;
  uint16_t am_time_left = amTimeLeft();
  uint16_t pm_time_left = pmTimeLeft();
  uint16_t mn_time_left = mnTimeLeft();
  Serial.println("Time left - AM: " + String(am_time_left) + " PM: " + String(pm_time_left) + " MN: " + String(mn_time_left));
  
  int dose_value = customVar.pillMatrix[today.period][today.day];

  if(doseMode == AM_ONLY){
    pm_time_left = 0;
    if(dose_value != DOSE_ACTIVE) am_time_left = 0;
  }else{
    if(today.period == 0 && dose_value != DOSE_ACTIVE) am_time_left = 0;
    if(today.period == 1 && dose_value != DOSE_ACTIVE){
      am_time_left = 0; 
      pm_time_left = 0;
    }
  }

  wakeByPeriod = 0;   // default to AM
  if (am_time_left > 0) {
    sleep_until = am_time_left;
    wakeByPeriod = 0;
  } else if (pm_time_left > 0 && doseMode == AM_PM) {
    sleep_until = pm_time_left;
    wakeByPeriod = 1;
  } else {
    // Midnight window
    sleep_until = mn_time_left;   // next real event is AM end
    wakeByPeriod = 3;             // next wake is AM
  }

  /*
  wakeByPeriod = 0;
  if(am_time_left > 0 ){
    sleep_until = am_time_left;
    wakeByPeriod = 0;
  }
  else if (pm_time_left > 0){
    wakeByPeriod = 1;
    sleep_until = pm_time_left;
  }else if (mn_time_left > 0){
    wakeByPeriod = 3;
    sleep_until = am_time_left;
  }
    */
  //advanceTimeByMinutes(sleep_until - 1);
  Serial.println("Next wake");
  Serial.println(sleep_until);
  if(simMode)sleep_until = 1;
  return sleep_until;
}

void advanceTimeByMinutes(int mins) {
    time_t t = mktime(&timeinfo);
    t += mins * 60;
    localtime_r(&t, &timeinfo);
}

void initaliseTimers(){
  
  //today = getDayPeriod(timeinfo);
  //currentDay = timeinfo.tm_wday;
  Serial.println("Today is day " + String(today.day) + " period " + String(today.period));
  Serial.println("Current day is " + String(currentDay));
  //When the system is first initalised we need to mark all past segments as missed
  if(initalisePillMatrix){
    initalisePillMatrixState();
    initalisePillMatrix = false;
    state = PB_HOME;
  }

  // If we are waking up due to a timer and the dose is not taken mark it as missed
  //the period value depends on the dose mode, in AM_ONLY mode we only care about the AM period, 
  //in AM_PM mode we need to check the current period that was marked for wake up
  if(wake == wakeReason.TIMER){
    if(wakeByPeriod != 3){
      if(doseMode == AM_ONLY){
        Serial.println("Marking dose as missed for day " + String(today.day) + " period " + String(today.period));
        customVar.pillMatrix[today.period][currentDay] = DOSE_MISSED;
      }else{
        customVar.pillMatrix[wakeByPeriod][currentDay] = DOSE_MISSED;
      }
    }else{
      //Check if the carousel needs to be reloaded, we do this at midnight 
      //so that the system is ready for the next day, if we miss this window for 
      //some reason we will check again when the user interacts with the device 
      DayPeriod out = getCarouselStatus();
      if(out.reload){
        matrixReset();
        segmentIndex = 0;
        state = PB_HOME;
      }
    }
  }
  
  getCarouselStatus();
  matrixRefresh();
  lastActivity = millis();
 
}

void initalisePillMatrixState(){
  int day, period;
  int currentIndex;

  if (doseMode == AM_PM) {
    currentIndex = today.day * 2;
    //Add one to the index for missed PM dose if we are in PM and the PM dose window has passed
    if(pmTimeLeft() == 0) currentIndex ++;
  } else { // AM_ONLY
    currentIndex = today.period * 7 + today.day;
  }
  //Add one to the index for missed AM dose if we are in PM and the AM dose window has passed
  if(amTimeLeft() == 0) currentIndex ++;

  for (int i = 0; i < currentIndex; i++) {
    if(doseMode == AM_PM){
      period = i % 2;   // 0 = AM, 1 = PM
      day    = i / 2;   // 0–6
    }else{
      period = i / 7;   // 0 or 1
      day  = i % 7;   // 0–
    }
    customVar.pillMatrix[period][day] = DOSE_MISSED;   // switched with the line belowe
  }

}

uint16_t amTimeLeft() {
  uint16_t now_minutes = toMins(timeinfo.tm_hour, timeinfo.tm_min);
  return  (am_window.end > now_minutes) ? am_window.end - now_minutes : 0;
}

uint16_t pmTimeLeft() {
  uint16_t now_minutes = toMins(timeinfo.tm_hour, timeinfo.tm_min);
  return (pm_window.end > now_minutes) ? pm_window.end - now_minutes : 0;
}

uint16_t mnTimeLeft() {
  uint16_t now_minutes = toMins(timeinfo.tm_hour, timeinfo.tm_min);
  return (midnight_window.end > now_minutes) ? midnight_window.end - now_minutes : 0;
}

const int LEEWAY = 1; // minutes

bool within(int now, int target) {
    return (now >= target - LEEWAY) && (now <= target + LEEWAY);
}

bool getTimeWithRetry() {

  if (simMode) {
    // Do NOT overwrite simulated time
    return true;
  }

  const int maxRetries = 20;     // 20 × 250ms = 5 seconds
  const int retryDelay = 250;

  for (int i = 0; i < maxRetries; i++) {
    if (getLocalTime(&timeinfo)) {
      return true;           // success
    }
    delay(retryDelay);
  }
 
  return false;                  // failed after retries
}

struct tm getLastSunday(struct tm now){
    struct tm s = now;
    int wday = now.tm_wday;   // 0 = Sun

    s.tm_mday -= wday;        // go back to Sunday
    mktime(&s);               // normalize

    return s;
}

int toMins(uint8_t h, uint8_t m) {
  return h * 60 + m;
}

DayPeriod getDayPeriod(struct tm now) {
  // Build start date
  struct tm start = {};
  start.tm_year = customVar.startYear;
  start.tm_mon  = customVar.startMonth;
  start.tm_mday = customVar.startDay;

  mktime(&start);
  mktime(&now);

  // Compute days since start
  time_t t1 = mktime(&start);
  time_t t2 = mktime(&now);

  long daysSinceStart = (t2 - t1) / 86400L;
  int weeksSinceStart = daysSinceStart / 7;

  DayPeriod out;

  // ✔ Correct pillbox day index
  out.day = daysSinceStart % 7;

  // ✔ Correct period logic
  if (doseMode == AM_ONLY) {
      out.period = weeksSinceStart & 1;
  } else {
      out.period = getCurrentPeriod();
  }
  return out;
}
/*
DayPeriod getDayPeriod(struct tm now, int addDays){
  
  int currentPeriod = getCurrentPeriod();
  if (addDays == 1) currentPeriod ^= 1;

  struct tm start = {};
  start.tm_year = customVar.startYear; // Year since 1900
  start.tm_mon  = customVar.startMonth;     // April
  start.tm_mday = customVar.startDay;

  DayPeriod out;

  // Build tomorrow's date
  struct tm tomorrow = now;
  tomorrow.tm_mday += addDays;
  mktime(&tomorrow);   // normalise rollover
//Serial.println(timeinfo.tm_wday);
  // Day number
  out.day = tomorrow.tm_wday;

  // Week number (0 or 1)
  time_t t1 = mktime(&start);
  time_t t2 = mktime(&tomorrow);

  long daysSinceStart = (t2 - t1) / 86400L;
  int weeksSinceStart = daysSinceStart / 7;
  if(doseMode == AM_ONLY){
    out.period = weeksSinceStart & 1;   // faster % 2
  }else{
    out.period = currentPeriod;
  }
  
  return out;
    
}
  */

int getCurrentPeriod(){
  uint16_t now_minutes = toMins(timeinfo.tm_hour, timeinfo.tm_min);
  return (now_minutes < pm_window.start) ? 0 : 1;
}

DayPeriod getSimulatedDayPeriod(int addDays){
    DayPeriod out;
    //out.day = (periodCounter + addDays) / 2;      // 0..13
    if (out.day == 7) out.day = 0;
   // out.period = (periodCounter + addDays) % 2;   // 0 = AM, 1 = PM
    return out;
}

//Temporary function to parse a date string in the format "DD MMM" and return a struct tm with the day, month, 
//and year set. The year is passed as an argument. This is used for testing the getCarouselIndex function with 
//different dates without having to change the RTC date.
struct tm parseDate(const char* dateStr, int year){
    struct tm t = {};
    char monthStr[10];
    int day;

    // Parse "05 May" or "5 May"
    sscanf(dateStr, "%d %s", &day, monthStr);

    // Convert month name to number
    const char* months[] = {
        "Jan","Feb","Mar","Apr","May","Jun",
        "Jul","Aug","Sep","Oct","Nov","Dec"
    };

    int month = -1;
    for (int i = 0; i < 12; i++) {
        if (strcasecmp(monthStr, months[i]) == 0) {
            month = i;
            break;
        }
    }

    if (month < 0) {
        Serial.println("Invalid month!");
        month = 0;
    }

    t.tm_mday = day;
    t.tm_mon  = month;          // 0–11
    t.tm_year = year - 1900;    // years since 1900

    // Normalize + compute weekday, yday, etc.
    mktime(&t);

    return t;
}

void getWakeReason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : 
      wake = wakeReason.EXT;
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1 : 
      wake = wakeReason.EXT;
      Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
      break;
    case ESP_SLEEP_WAKEUP_TIMER :
      wake = wakeReason.TIMER;
      Serial.println("Wakeup caused by timer"); 
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default :
      wake = wakeReason.BOOT;
      Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); 
      break;
    } 
}

#pragma endregion TIME_HELPER

#pragma region MATRIX_HELPERS

#pragma region DISPLAY_HELPERS

void messageHeader(){

  display.setTextColor(GxEPD_BLACK);    // Set the text color to black

  display.setFont(&FreeSansBold9);
  printCenter("Smart Pill Carousel", 12);
  display.setFont(&FreeSansBold7pt);
  printCenter("apps4leisure", 25);
  display.drawBitmap(168, 18, copyright_8x8, 8, 8, GxEPD_BLACK);
 
  drawBatteryStatus(80);
  display.drawBitmap(8, 5, wifi_off,24, 24, GxEPD_BLACK);
}

void buildMainDisplay(){
  messageHeader();

  //display.drawBitmap(8, 5,wifi_connecting, 24, 24, GxEPD_BLACK);
  
  drawBatteryStatus(5);

  drawMatrix();

  //display.drawRect(175, MAIN_TOP + 51, 2, 48, GxEPD_BLACK);
  //display.drawRect(62, MAIN_TOP + 50, 2, 48, GxEPD_BLACK);

  //drawCupStatus(getCupState());
  //drawLidStatus(getLidState());
  drawState("IDLE");

  display.display();
}

void wifiSetupMessage(){

  messageHeader();

  display.setFont(&FreeSansBold9);
  printCenter("Please setup WiFi",50);
  printCenter("using the companion app",65);

  display.drawBitmap(105, 75, wifi_setup, 48, 48, GxEPD_WHITE, GxEPD_BLACK);

  display.display();
 
}

void dosesSetupMessage(){
 
  display.setFont(&FreeSansBold7pt);
  printCenter("Set doses per day?", 50);
  display.setCursor(25, 75);
  display.println("Button A: One (Wk1/Wk2)");
  display.setCursor(25, 95);
  display.println("Button B: Two (AM/PM)");

 // display.display();
}

void wifiRefresh(WiFiState state){
 Serial.println("Updating WiFi Icon: " + String(state));
  const uint8_t* wifiIcon;
  switch (state) {
    case WIFI_IDLE:
      wifiIcon = wifi_off;
      break;
    case WIFI_CONNECTING:
      wifiIcon = wifi_connecting;
      break;
    case WIFI_CONNECTED:
      wifiIcon = wifi_connected;
      break;
    case WIFI_FAILED:
      wifiIcon = wifi_off;
      break;
    case WIFI_SLEEP:
      wifiIcon = wifi_sleep;
      break;
  }
  display.firstPage();
  do{
    display.setPartialWindow(0, 0, 40, 40);
    display.drawBitmap(8, 5, wifiIcon,24, 24, GxEPD_BLACK);
  } while (display.nextPage());
  display.firstPage();
}

void batteryRefresh(){
  display.firstPage();
  do{
    display.setPartialWindow(200, 0, 40, 40);
    drawBatteryStatus(80);
  } while (display.nextPage());
  display.firstPage();
}

void stateRefresh(const String &stateString){
  display.setFont(&FreeSansBold7pt);
  display.firstPage();
  do{
    display.setPartialWindow(0, MAIN_TOP + 60, 160, 40);
    drawState(stateString);
  } while (display.nextPage());
  display.firstPage();
}

void matrixRefresh(){
  display.firstPage();
  do{
    display.setPartialWindow(0, MAIN_TOP, display.width(), 52);
    drawMatrix();
  } while (display.nextPage());
  display.firstPage();
  EEPROM.put(0, customVar);
  EEPROM.commit();
  if(!sendPillMatrix()) ESP.restart();
}

void drawBatteryStatus(int z){
/*
const int fillWidth = (batteryLevel * barWidth) / 100;

// Draw battery fill
display.fillRect(x + 1, y + 1, fillWidth - 3, barHeight, GxEPD_BLACK);

// Draw percentage text below the bar
display.setTextColor(GxEPD_BLACK);
display.setFont(&FreeMonoBold9pt7b);   // or whatever font you're using


int textX = x;                 // align left with the bar
int textY = y + barHeight + 15; // 15px below the bar

display.setCursor(textX, textY);
display.printf("%d%%", batteryLevel);
*/

  const int barWidth = 18;
  const int barHeight = 8;
  const int  y = 7;
  const int x = display.width() - barWidth - 11; // Position on the right side

  display.drawBitmap(218, 0, battery_empty, 24, 24, GxEPD_BLACK);
  //double actualVoltage = ReadVoltage(analogRead(BATTERY_REF_PIN)) * 2.07; 
  //int actualePercent = calcPercentage(actualVoltage, 3.00, 3.7);
  //if (actualePercent > 100 ) actualePercent = 100;

  int raw = readADC_Averaged(BATTERY_REF_PIN, 64);
  double voltage = ReadVoltage(raw);
  
  int batteryLevel = 100; // Replace with actual battery level reading

  const int fillWidth = (batteryLevel * barWidth) / 100;
  // Draw battery fill
  display.fillRect(x + 1, y + 1, fillWidth - 3, barHeight, GxEPD_BLACK);
  //display.setFont(&FreeSansBold6pt7b);


// Your battery text
String batteryText = "100%";  // try "1%", "20%", etc.

// Measure text
int16_t x1, y1;
uint16_t w, h;

display.setFont(&FreeSansBold5pt7b); // or your chosen font
display.getTextBounds(batteryText, 0, 0, &x1, &y1, &w, &h);

int iconX = 216;
int iconY = 28;
int iconW = 24;

// Calculate centred X
int textX = iconX + (iconW - w) / 2;

// Position under icon (adjust spacing as needed)
//int textY = iconY + 24 + 10;

// Draw text
display.setCursor(textX, 28);
display.print(batteryText);

  /*
  display.setCursor(215, 28); // Add 
  display.print(batteryLevel);
  display.print("%");  
    */
    // Optionally, add a percentage text
    //display.setFont(&FreeSansBold6pt7b);
    //display.setCursor(x - 30, y + barHeight); // Position to the left of the battery icon
    //display.print(batteryLevel);
    //display.print("%");
  
}

void drawStatus(){
  display.setFont(&FreeSansBold7pt);
  
  //drawCupStatus(false);
  //drawLidStatus(false);

  drawState("IDLE");
  
}

void drawState(const String &stateString){
 // printCentre("DISPENSING", MAIN_TOP + 70);
  printCenter(stateString, MAIN_TOP + 70, -5);
}

void printCenter(const String &text, int y, int offset){
  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(text, 0, 0, &tbx, &tby, &tbw, &tbh);
  int x = ((display.width() - tbw) / 2) - tbx + offset;
  display.setCursor(x, y);
  display.print(text);
}

void matrixReset(){
  Serial.println("Reset pressed");
  for (int p = 0; p < 2; p++){
    for (int d = 0; d < 7; d++){
      customVar.pillMatrix[p][d] = DOSE_NONE;
    }
  }
}

void drawMatrix(){

  display.drawRect(0, MAIN_TOP + 5, 260, 2, GxEPD_BLACK);
  display.drawRect(0, MAIN_TOP + 50, 260, 2, GxEPD_BLACK);
  display.drawRect(34, MAIN_TOP + 5, 2, 45, GxEPD_BLACK);

  display.setFont(&FreeMonoBold7pt7b);
  //display.setCursor(44, MAIN_TOP);
  display.setTextColor(GxEPD_BLACK);
  for (int i = 0; i < 7; i++){
    //we don't have the current day of the week when we boot from deep sleep, 
    //so we check if currentDay is set to a valid value before trying to highlight it
    if (i == currentDay && i > -1){
      display.drawRect(44 + (30 * i), MAIN_TOP + 5, 25, 2, GxEPD_WHITE);
    }
    display.setCursor(44 + (30 * i), MAIN_TOP);
    display.print(dayName[i]);
  }

  // Define the segments and label the rows AM/PM or W1/W2 depending on the mode
  String text ="";
  for (int period = 0; period < 2; period++){
    display.setCursor(4, MAIN_TOP + (21 * (period + 1)));
    display.setTextColor(GxEPD_BLACK);
    if (doseMode == AM_PM) {
      text = (period == 0) ? "AM" : "PM";
    }else{
      text = (period == 0) ? "WK1" : "WK2";
    }
    display.print(text);

    int y =  MAIN_TOP + 9 + (20 * period);
    for (int day = 0; day < 7; day++){
      int x = 45 + (30 * day);
      if (customVar.pillMatrix[period][day] == DOSE_NONE){
        display.drawBitmap(x, y, capsule_full, 20, 20, GxEPD_BLACK);
      } else if  (customVar.pillMatrix[period][day] == DOSE_TAKEN){
        display.drawBitmap(x, y, capsule_taken, 20, 20, GxEPD_BLACK);
      }else if (customVar.pillMatrix[period][day] == DOSE_MISSED){
        display.drawBitmap(x, y, capsule_missed, 20, 20, GxEPD_BLACK);
      } if (customVar.pillMatrix[period][day] == DOSE_ACTIVE){
        display.drawBitmap(x, y, current_pill, 20, 20, GxEPD_BLACK);
      }
    }
  }

  int currentPeriod = 1;//timeFunctions.getCurrentPeriod();
  if(currentPeriod == 1){
    display.drawRect(2, MAIN_TOP + 25, 24, 2, GxEPD_BLACK);
  }else{
    display.drawRect(2, MAIN_TOP + 40, 24, 2, GxEPD_BLACK);
  }

}

#pragma endregion

#pragma region BATTERY_HELPER

int readADC_Averaged(int pin, int samples) {
  uint32_t total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delayMicroseconds(200); // small settling delay
  }
  return total / samples;
}

int voltageToPercent(float v) {
  for (int i = 0; i < sizeof(socTable)/sizeof(socTable[0]) - 1; i++) {
    if (v >= socTable[i+1].voltage) {
      float v1 = socTable[i].voltage;
      float v2 = socTable[i+1].voltage;
      int p1 = socTable[i].percent;
      int p2 = socTable[i+1].percent;

      float t = (v - v2) / (v1 - v2);
      return p2 + t * (p1 - p2);
    }
  }
  return 0;
}

// Convert averaged ADC reading to battery voltage
double ReadVoltage(int pin) {
  const double R1 = 100000.0;  // top resistor
  const double R2 = 100000.0;  // bottom resistor
  const double ADC_REF = 3.3;  // ESP32-S3 calibrated reference
  const double ADC_MAX = 4095.0;

  int raw = readADC_Averaged(pin, 64);

  // Voltage at ADC pin
  double v_adc = (raw / ADC_MAX) * ADC_REF;

  // Battery voltage after divider
  double v_batt = v_adc * ((R1 + R2) / R2);

  return v_batt;
}

int calcPercentage(float actual, float min, float max){
  //Serial.print("min:");
  //Serial.println(min);
  // Serial.print("max:");
  //Serial.println(max);
  int result = (actual - min) / (max - min) * 100;
  if (result < 0) result = 0;
  if (result > 100) result = 100;
  return result;
}
#pragma endregion




/*
void wifiLoop() {

  switch (wifiState) {

    case WIFI_CONNECTING:
      Serial.print(".");
      if (WiFi.status() == WL_CONNECTED) {
        wifiState = WIFI_CONNECTED;
        Serial.println("Connected....");
        //refresh the display to show connected status
        wifiSetRTC();
        initaliseTimers();
        lastActivity = millis();
        if(dispence) dispenseA();
        // Set the RTC from wifi time and refresh the display to show the current day
      
        // Refresh the display to show the current day
        //matrixRefresh();
      } else if (millis() - wifiStartTime > WIFI_TIMEOUT) {
        WiFi.disconnect(true);
        wifiState = WIFI_FAILED;
      }
      break;

    case WIFI_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        wifiState = WIFI_DISCONNECTED;
        wifiRefresh(wifiState);
      }
      break;

    case WIFI_DISCONNECTED:
      // try again immediately or after a delay
      wifiRequest();
      break;

    case WIFI_IDLE:
    case WIFI_FAILED:
      // do nothing until wifiRequest() is called
      break;
  }
}
  */

  /*
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)){
    struct tm setupDate = getLastSunday(timeinfo);
    customVar.startYear = setupDate.tm_year;
    customVar.startMonth = setupDate.tm_mon;
    customVar.startDay = setupDate.tm_mday;
    EEPROM.put(0, customVar); 
    EEPROM.commit();
  }
  */
  //getCarouselSegment(AM_ONLY);

/*
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)){

    customVar.setupDate = getLastSunday(timeinfo);
    EEPROM.put(0, customVar); 
    EEPROM.commit();
  }
*/
  /*
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)){
    currentDay = timeinfo.tm_wday;
    int hour = timeinfo.tm_hour;
    int min = timeinfo.tm_min;

    struct tm tmpDate{0};
      tmpDate.tm_year = 126; // Year since 1900
      tmpDate.tm_mon = 3;   
      tmpDate.tm_mday = 31;            // Day of the month (1-31)
    Serial.println(dateDiff(customVar.setupDate, tmpDate));
    Serial.println(dateDiff(customVar.setupDate, tmpDate)/7);
    Serial.println(timeinfo.tm_year);
    /*
    dateStruct.tm_year = timeinfo.tm_year; // Year since 1900
    dateStruct.tm_mon = timeinfo.tm_mon;            // Month (0-11)`
    dateStruct.tm_mday = timeinfo.tm_mday;           // Day of the month (1-31)
    dateStruct.tm_hour = 0;
    dateStruct.tm_min = 0;
    dateStruct.tm_sec = 0;
    customVar.setupDate = dateStruct;
    EEPROM.put(0, customVar); 
    EEPROM.commit();
    */

    //Serial.print("WeekDay: "); Serial.println(currentDay);
    //Serial.print("Hour: "); Serial.println(hour);
    //Serial.print("Minute: "); Serial.println(min);
  //}

 //}



/* 
  Serial.println(state);
  if(state == PB_MODE_CONFIG){
    doseMode = AM_ONLY;
    state = PB_ADD_DEVICE;
    customVar.doseMode = doseMode;
    EEPROM.put(0, customVar);
    EEPROM.commit();
    return;
  } */
/*
  stepper.setMaxSpeed(1200);
  stepper.setAcceleration(300);
  stepper.runSpeed();
  stepper.move(STEPS_PER_SEG);

  segmentCounter ++;

  if (segmentCounter > 14) segmentCounter = 1;

  int day, row;
  getDayRow(segmentCounter, AM_ONLY, day, row);

  Serial.print("Day: ");
  Serial.print(day);
  Serial.print(", Row: ");
  Serial.println(row);
  state = PB_DISPENSE;
  */

/*
void home() {

  stepper.setMaxSpeed(600);
  stepper.setAcceleration(300);

  Serial.println("Homing...");
  state = PB_HOME;
}
  */
/*
void reverse() {
  stepper.setMaxSpeed(1200);
  stepper.setAcceleration(300);
  stepper.runSpeed();
  stepper.move(-STEPS_PER_SEG);

  segmentCounter --;

  if (segmentCounter < 1) segmentCounter = 14;

  int day, row;
  getDayRow(segmentCounter, AM_ONLY, day, row);

  Serial.print("Day: ");
  Serial.print(day);
  Serial.print(", Row: ");
  Serial.println(row);
  state = PB_REVERSE;
} 


void getDayRow(int n, int &day, int &row) {
  int index = n - 1; // Convert to 0-based index

  if (doseMode == AM_ONLY) {
    day = (index % 7) + 1;
    row = (index / 7) + 1;
  } else{
    day = (index / 2) + 1;
    row = (index % 2) + 1;
  }
}
*/


/*
void startWiFi() {
  
  wifiState = WIFI_CONNECTING;
  wifiStartTime = millis();

  WiFi.mode(WIFI_STA);
  WiFi.begin("PLUSNET-H7A8XX","7213982454p");

}

void updateWiFi() {
  switch (wifiState) {

    case WIFI_CONNECTING:

      if (WiFi.status() == WL_CONNECTED) {
          wifiState = WIFI_CONNECTED;
      } 
      else if (millis() - wifiStartTime > WIFI_TIMEOUT) {
          wifiState = WIFI_FAILED;
          WiFi.disconnect(true);
      }
      break;
    default:
      break;
  }
}

*/

/*
void drawCupStatus(bool state){
  display.setFont(&FreeSansBold7pt7b);
  String text = "IN";
  if (!state){
    text = "OUT";
    display.setCursor(13, 112); // Add 
    display.print("!");
  } else {
    display.setCursor(10, 112); // Add 
    display.print("O");
  }

  display.drawRoundRect(6, 99, 19, 19, 0, GxEPD_BLACK);
  display.drawRoundRect(7, 100, 17, 17, 0, GxEPD_BLACK);

  display.setCursor(30, MAIN_TOP + 70); // Add 
  display.print(text); 
 
}

void drawLidStatus(bool state){
  display.setFont(&FreeSansBold7pt7b);
  String text = "SHUT";
  if (!state){
    text = "OPEN";
    display.fillRect(210 - 24, 98, 15, 2, GxEPD_BLACK);
  } 
  
  display.drawRoundRect(210 - 24, 102, 15, 15, 0, GxEPD_BLACK);
  display.drawRoundRect(211 - 24, 103, 13, 13, 0, GxEPD_BLACK);
  display.drawRoundRect(211 - 24, 105, 13, 2, 0, GxEPD_BLACK);

  display.setCursor(205, MAIN_TOP + 70); // Add 
  display.print(text);  
  
}
*/


/*
void updateDisplayIcon(WiFiState state) {
  Serial.println("Updating WiFi Icon: " + String(state));
  uint8_t* wifiIcon;
  switch (state) {
    case WIFI_IDLE:
      wifiIcon = wifi_off;
      
      break;
    case WIFI_CONNECTING:
      display.drawBitmap(8, 5, wifi_connected,20, 20, GxEPD_BLACK);
      break;
    case WIFI_CONNECTED:
      wifiRefresh(true);
      break;
    case WIFI_FAILED:
      display.drawBitmap(8, 5, wifi_off,20, 20, GxEPD_BLACK);
      break;
  }
} 
  */
/*
bool wifiConnect(){
  WiFi.begin("PLUSNET-H7A8XX","7213982454p");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("WiFi Connected");
    Serial.println(WiFi.localIP());
    return true;
  }
  return false;
}
*/


/*
time_t getNextWakeTime(int hour, int minute)
{
    struct tm now;
    getLocalTime(&now);   // your RTC/NTP/local time source

    struct tm wake = now;
    wake.tm_hour = hour;
    wake.tm_min  = minute;
    wake.tm_sec  = 0;

    time_t wakeEpoch = mktime(&wake);
    time_t nowEpoch  = mktime(&now);

    // If today's wake time has passed, move to tomorrow
    if (wakeEpoch <= nowEpoch)
    {
        wake.tm_mday += 1;
        wakeEpoch = mktime(&wake);   // normalize again (DST-safe)
    }

    return wakeEpoch;
}

uint64_t getSleepTime(DoseWindow window) {
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    // Create target time (00:20 today)
    struct tm target = timeinfo;
    target.tm_hour = window.end / 60;
    target.tm_min  = window.end % 60 + 20;
    target.tm_sec  = 0;

    time_t target_time = mktime(&target);

    // If we've already passed today's 00:20 → go to tomorrow
    if (difftime(target_time, now) <= 0) {
        target.tm_mday += 1;
        target_time = mktime(&target);
    }

    // Difference in seconds
    double seconds = difftime(target_time, now);

    return (uint64_t)(seconds * uS_TO_S_FACTOR);
}

DoseResult getNextDose(int nowMins) {

  int am = nextAM(nowMins);

  if (doseMode== AM_ONLY) {
    if (am < 24*60) return TODAY_AM;
    return TOMORROW_AM;
  }

  // MODE_BOTH
  int pm = nextPM(nowMins);

  if (am <= pm) {
    if (am < 24*60) return TODAY_AM;
    return TOMORROW_AM;
  }

  // PM is sooner
  if (pm < 24*60) return TODAY_PM;

  // PM rolled over → next is tomorrow AM
  return TOMORROW_AM;
}

int nextPM(int nowMins) {
  //int pmStart = PM_START;
  //int pmEnd   = PM_END;

  if (nowMins < pm_window.start) return pm_window.start;          // before PM
  if (nowMins < pm_window.end)   return nowMins;          // inside PM
  return pm_window.start + 24*60;                         // tomorrow PM
}

int nextAM(int nowMins) {
  //int amStart = AM_START;
  //int amEnd   = AM_END;

  if (nowMins < am_window.start) return am_window.start;          // before AM
  if (nowMins < am_window.end)   return nowMins;          // inside AM
  return am_window.start + 24*60;                         // tomorrow AMU
}

int dateDiff(struct tm start, struct tm end) {

  int dayNumber = end.tm_wday;
 
  time_t t1 = mktime(&start);
  time_t t2 = mktime(&end);

  return ((t2 - t1) / 86400)/7;
}
*/
