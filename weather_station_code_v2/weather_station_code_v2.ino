// ============================================================================
// PORTABLE WEATHER STATION v2  — standalone program
// Sophie LV Scopazzi
// 
// Made with assistance of Claude Opus 4.8 High
//
// Hardware (Feather RP2040, same RTC + OLED as SeaLabMOS):
//   - DS3231 RTC          I2C 0x68   INT/SQW -> GPIO25 (wakes the CPU)
//   - SH1107 OLED Wing    I2C 0x3C
//   - Sensirion SHT31-D   I2C 0x44   air temperature + relative humidity
//   - Infineon DPS310     I2C 0x77   barometric pressure + temperature
//   - microSD (SdFat)     SPI1, CS 23
//
// Two modes (set `deviceMode` in USER SETTINGS):
//   MODE_SLEEP      Battery. Sleeps on the RTC alarm between samples, logs every
//                   WS_WAIT_MINUTES (clock-aligned), OLED wakes briefly per
//                   sample then sleeps.
//   MODE_CONTINUOUS Powered. Never sleeps, logs every WS_CONTINUOUS_INTERVAL_MS
//                   (default 10 s), OLED stays on.
//
// NOTE ON POWER (MODE_SLEEP): __wfi() is a LIGHT sleep — the RP2040 core halts
// but clocks and peripherals keep running, so the chip still draws several mA
// between samples. Same trade-off SeaLabMOS makes. True low power on RP2040
// needs dormant/sleep mode (more involved); this is the practical version. The
// long duty cycle is what saves the battery, not the sleep depth.
//
// Differs from v1: no GPS (RTC provides time); SHT31 instead of SHT45; DPS310
// instead of BMP390; ambient temp comes from the SHT31 (better for air temp
// than a baro chip, which self-heats), DPS310 temp logged as a cross-check;
// a missing sensor no longer halts the program (only SD failure does) so one
// dead sensor can't stop logging. Dew point (Magnus, from SHT31 T+RH) is
// computed on-board and logged as its own column — it's the moisture variable
// that survives siting/temperature offsets, unlike RH.
//
// BROWNOUT PROTECTION (new): the Mare Island deployment ran the pack down to
// brownout — the rail collapsed at ~3.43-3.44 V indicated, the RTC/GPS time
// froze, and the system spent its last minutes writing 385 garbage rows with
// the SD card exposed mid-write. To prevent that: every sample, after reading
// vBat, the firmware checks against WS_LOWBATT_CUTOFF_V. A single low reading
// is confirmed by re-reading several times over a few seconds (so a momentary
// sag — fan spin-up, SD write burst — can't trip it). On a confirmed low
// battery the firmware writes one final marker line, closes the file cleanly,
// disarms the RTC alarm, shows a LOW BATT screen briefly, turns the OLED off,
// and halts. The last line of a brownout file is:
//   LOW_BATT_SHUTDOWN,<ISO timestamp>,<vBat>
// (3 fields, not 9 — easy to detect when parsing; everything above it is a
// clean, complete row.)
// ============================================================================

#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <RTClib.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_DPS310.h>

#include "pico/stdlib.h"
#include "hardware/sync.h"   // __wfi()

// ===================== USER SETTINGS =====================
// Operating mode — pick one:
//   MODE_SLEEP      Battery. RTC-alarm sleep between samples (__wfi). Logs every
//                   WS_WAIT_MINUTES, clock-aligned. OLED wakes briefly per sample
//                   then sleeps. Low(er) power.
//   MODE_CONTINUOUS Powered. Never sleeps. Logs every WS_CONTINUOUS_INTERVAL_MS.
//                   OLED stays ON the whole time. NOT for battery deployment.
#define MODE_SLEEP        0
#define MODE_CONTINUOUS   1
int     deviceMode      = MODE_CONTINUOUS;

#define WS_WAIT_MINUTES          10       // MODE_SLEEP interval, minutes (clock-aligned)
#define WS_CONTINUOUS_INTERVAL_MS 1000UL // MODE_CONTINUOUS interval (10 s)
#define WS_DISPLAY_ON_MS         5000UL   // MODE_SLEEP: how long the OLED shows each sample
int     timeZone        = -7;       // local offset from UTC, for compile-time RTC set

// Battery divider correction. MEASURE: read vBat from serial while reading the
// pack with a multimeter, then tune this until they match. Corrects for not
// knowing the exact divider resistor values. SeaLabMOS uses ~1.03–1.07 per
// build; 1.053 is its average and a fine starting point.
#define VBAT_CORRECTION  1.064 // measured with multimeter

// ---- Brownout protection ----
// Mare Island: rail collapse began at ~3.43-3.44 V indicated (AP2112K dropout
// under ~100+ mA load), with peripherals flaking below ~3.45. The cutoff sits
// just above that so the final write happens while the rail is still solid.
// Bench-verify per unit (step a supply down in 50 mV increments under the real
// load) and raise the cutoff if your unit collapses higher.
#define WS_LOWBATT_CUTOFF_V       3.50f   // confirmed below this -> safe shutdown
#define WS_LOWBATT_CONFIRM_READS  5       // consecutive low re-reads to confirm
#define WS_LOWBATT_CONFIRM_MS     1000UL  // spacing between confirm re-reads
// =========================================================

// ---- Pins / addresses ----
#define SD_CS_PIN     23
#define RTC_INT_PIN   25            // DS3231 INT/SQW — must be wired here
#define OLED_ADDR     0x3C
#define SHT31_ADDR    0x44
#define DPS310_ADDR   0x77
#define VBATPIN       A2            // same as SeaLabMOS BATTV_PIN (v1 used A3)
#define DISPLAY_BTN_PIN  4   // pick a free GPIO

// ---- Objects ----
SdFat        SD;
File32       dataFile;
SdSpiConfig  config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

RTC_DS3231       rtc;
Adafruit_SH1107  display = Adafruit_SH1107(64, 128, &Wire);
Adafruit_SHT31   sht31   = Adafruit_SHT31();
Adafruit_DPS310  dps;

// ---- Wake flag (set by the RTC interrupt) ----
volatile bool rtcAlarmFired = false;
void rtcWakeISR() { rtcAlarmFired = true; }

// ---- Readings (999.0 = no/failed read) ----
float tempC       = 999.0;   // SHT31 air temperature, degC
float tempF       = 999.0;
float humidity    = 999.0;   // SHT31 relative humidity, %
float dewPointC   = 999.0;   // computed from SHT31 T + RH (Magnus), degC
float pressurehPa = 999.0;   // DPS310 station pressure, hPa (not sea-level reduced)
float dpsTempC    = 999.0;   // DPS310 temperature, degC (cross-check)
float rp2040TempC = 999.0;   // RP2040 internal DIE temp, degC (diagnostic, NOT air temp)
float vBat        = 0.0;

String timestamp_filename = "";   // YYYY-MM-DD (one file per day)

bool sht31Ok = false;
bool dpsOk   = false;

// ----------------------------------------------------------------------------
// OLED power-save (raw SH1107 commands, same as SeaLabMOS)
// ----------------------------------------------------------------------------
void oledSleep() {                 // 0xAE = display OFF
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00); Wire.write(0xAE);
  Wire.endTransmission();
}
void oledWake() {                  // 0xAF = display ON
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00); Wire.write(0xAF);
  Wire.endTransmission();
}

// ----------------------------------------------------------------------------
// RTC helpers
// ----------------------------------------------------------------------------
// Set the RTC from the sketch's compile time, converted to UTC. Only used when
// the coin cell has died (lostPower). Compile time is approximate — for an
// accurate clock, recompile right before flashing or set it from a known source.
void setRtcCompileTimeUTC() {
  DateTime compiled = DateTime(F(__DATE__), F(__TIME__));
  rtc.adjust(DateTime(compiled.unixtime() - (timeZone * 3600)));
}

// Next clock-aligned alarm time (e.g. :00, :10, :20 ...). Day/month rollover is
// naive, but harmless: DS3231_A1_Minute only matches minutes + seconds, so the
// date fields never affect when the alarm fires.
DateTime getNextAlarm(DateTime now, uint8_t waitMin) {
  uint8_t  nextMinute = ((now.minute() / waitMin) + 1) * waitMin;
  uint8_t  nextHour   = now.hour();
  uint8_t  nextDay    = now.day();
  uint8_t  nextMonth  = now.month();
  uint16_t nextYear   = now.year();
  if (nextMinute >= 60) {
    nextMinute %= 60;
    nextHour++;
    if (nextHour >= 24) { nextHour = 0; nextDay++; }
  }
  return DateTime(nextYear, nextMonth, nextDay, nextHour, nextMinute, 0);
}

void armNextAlarm() {
  rtcAlarmFired = false;
  rtc.clearAlarm(1);
  DateTime next = getNextAlarm(rtc.now(), WS_WAIT_MINUTES);
  rtc.setAlarm1(next, DS3231_A1_Minute);
  Serial.print("Next alarm @ ");
  Serial.println(next.timestamp(DateTime::TIMESTAMP_FULL));
}

// ----------------------------------------------------------------------------
// SD card
// ----------------------------------------------------------------------------
// Show the SD-failure screen and halt. No safe way to keep logging without
// storage, so we stop and wait for a restart.
void sdCardFailed() {
  Serial.println("Card failed, or not present :(");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("    SD card FAILED");
  display.println("");
  display.println("  RESTART or PLUG IN");
  display.println("");
  display.println("If it still doesn't");
  display.println("work you may need to");
  display.println("REFORMAT the card");
  display.println("");
  display.println("Try exFAT");
  display.display();
  while (1) {}   // halt until fixed and restarted
}

void setupSD() {
  delay(100);
  Serial.println("Initializing SD card...");
  if (!SD.begin(config)) sdCardFailed();
  Serial.println("Card initialized!");
}

// Set today's filename and write the header if the file is new. Called every
// wake so it rolls over to a fresh file at midnight automatically.
void ensureDailyFile() {
  DateTime now = rtc.now();
  char d[11];
  snprintf(d, sizeof(d), "%04d-%02d-%02d", now.year(), now.month(), now.day());
  timestamp_filename = String(d);

  dataFile.open((timestamp_filename + ".csv").c_str(), O_WRITE | O_CREAT | O_APPEND);
  if (!dataFile) sdCardFailed();
  if (dataFile.fileSize() == 0) {
    dataFile.println("PORTABLE WEATHER STATION v2");
    dataFile.println("time,tempC,tempF,humidity,dewPointC,pressure_hPa,dpsTempC,rp2040TempC,vBat");
  }
  dataFile.close();
}

void writeRow() {
  DateTime now = rtc.now();
  dataFile.open((timestamp_filename + ".csv").c_str(), O_WRITE | O_CREAT | O_APPEND);
  if (!dataFile) { sdCardFailed(); return; }
  dataFile.print(now.timestamp(DateTime::TIMESTAMP_FULL)); dataFile.print(',');
  dataFile.print(tempC, 2);       dataFile.print(',');
  dataFile.print(tempF, 2);       dataFile.print(',');
  dataFile.print(humidity, 2);    dataFile.print(',');
  dataFile.print(dewPointC, 2);   dataFile.print(',');
  dataFile.print(pressurehPa, 2); dataFile.print(',');
  dataFile.print(dpsTempC, 2);    dataFile.print(',');
  dataFile.print(rp2040TempC, 2); dataFile.print(',');
  dataFile.println(vBat, 2);
  dataFile.close();
}

// ----------------------------------------------------------------------------
// Sensors
// ----------------------------------------------------------------------------
void setupSHT31() {
  sht31Ok = sht31.begin(SHT31_ADDR);
  if (sht31Ok) {
    // Heater OFF — only for deliberate condensation burn-off, never continuous;
    // leaving it on biases temperature high (the SHT45 heater trap).
    sht31.heater(false);
  }
  Serial.println(sht31Ok ? "Found SHT31" : "SHT31 NOT FOUND");
}

void setupDPS310() {
  dpsOk = dps.begin_I2C(DPS310_ADDR);
  if (dpsOk) {
    // ODR rule: per-channel measurement time must fit one 1/rate period or the
    // DPS310 reports ODR errors. 64x oversampling ~104 ms; at 1 Hz that's always
    // safe and gives best precision. (Continuous mode; the RP2040 WFI dominates
    // power, so one-shot wouldn't move the needle here.)
    dps.configurePressure(DPS310_1HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_1HZ, DPS310_64SAMPLES);
  }
  Serial.println(dpsOk ? "Found DPS310" : "DPS310 NOT FOUND");
}

void readSensors() {
  if (sht31Ok) {
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    if (!isnan(t)) { tempC = t; tempF = t * 9.0 / 5.0 + 32.0; }
    if (!isnan(h)) { humidity = h; }
    // Dew point (Magnus formula, Sonntag constants). Only valid when both T
    // and RH are fresh and RH is physical; otherwise leave the 999 sentinel.
    if (tempC != 999.0 && humidity != 999.0 && humidity > 0.0 && humidity <= 100.0) {
      float g  = logf(humidity / 100.0f) + (17.62f * tempC) / (243.12f + tempC);
      dewPointC = (243.12f * g) / (17.62f - g);
    } else {
      dewPointC = 999.0;
    }
  }
  if (dpsOk && dps.temperatureAvailable() && dps.pressureAvailable()) {
    sensors_event_t te, pe;
    if (dps.getEvents(&te, &pe)) {
      dpsTempC    = te.temperature;
      pressurehPa = pe.pressure;     // hPa
    }
  }
}

void readVBat() {
  // 12-bit ADC, 1:1 divider — same as SeaLabMOS readBatteryVoltage().
  delay(5);                          // stability
  float v = analogRead(VBATPIN);
  v /= 4095.0;                       // 12-bit raw -> 0..3.3 V
  v *= 3.3;                          // voltage at the pin
  v *= 2.0;                          // undo the 1:1 divider
  v *= VBAT_CORRECTION;              // measured divider correction
  vBat = v;
}

void readChipTemp() {
  // RP2040 internal temperature sensor (ADC4). This is DIE temperature, not air
  // temperature — a diagnostic for self-heating / electronics health (handy in
  // MODE_CONTINUOUS where the OLED + always-on core keep the board warm), NOT a
  // weather variable. The SHT31 still owns air temp.
  //
  // analogReadTemp() (Earle Philhower arduino-pico core) selects ADC4, reads it,
  // and converts with the datasheet formula. It leaves the ADC input on ch4, but
  // the next analogRead()/readVBat() reselects A2, so vBat is unaffected — keep
  // readVBat() *before* this call (as sampleLog does) and there's no interaction.
  //
  // Accuracy caveat: the on-die sensor is uncalibrated. The 0.706 V @ 27 degC
  // reference has real part-to-part spread, so absolute error is ~±2-3 degC.
  // Trust the trend, not the last digit. (vref defaults to 3.3 V = the AP2112K
  // rail; near brownout the rail sags and this reading skews, but we shut down
  // there anyway.)
  float t = analogReadTemp();        // degC
  if (!isnan(t)) rp2040TempC = t;
}

// ----------------------------------------------------------------------------
// Brownout protection
// ----------------------------------------------------------------------------
// Write the shutdown marker and stop everything, cleanly, while the rail is
// still healthy. Called only after a CONFIRMED low battery (see below).
void lowBattShutdown() {
  Serial.print("LOW BATTERY — safe shutdown at vBat=");
  Serial.println(vBat, 2);

  // Final marker line. Deliberately 3 fields (not 9) so any parser can spot it;
  // every row above it is complete and clean.
  DateTime now = rtc.now();
  dataFile.open((timestamp_filename + ".csv").c_str(), O_WRITE | O_CREAT | O_APPEND);
  if (dataFile) {
    dataFile.print("LOW_BATT_SHUTDOWN,");
    dataFile.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    dataFile.print(',');
    dataFile.println(vBat, 2);
    dataFile.close();              // close = flush; SD is now safe
  }

  // No more wakes: disarm the alarm and ignore the INT pin.
  detachInterrupt(digitalPinToInterrupt(RTC_INT_PIN));
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);

  // Tell whoever finds it what happened, then kill the display to stop wasting
  // what little is left of the pack.
  oledWake();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("   LOW BATTERY");
  display.println("");
  display.print("vBat: "); display.println(vBat, 2);
  display.println("");
  display.println("Logging stopped.");
  display.println("File closed safely.");
  display.println("");
  display.println("CHARGE & RESTART");
  display.display();
  delay(10000);
  oledSleep();

  // Halt. __wfi in a loop instead of while(1){} so the dying core at least
  // isn't spinning at full draw.
  while (1) { __wfi(); }
}

// Check the just-read vBat against the cutoff. A single low reading is NOT
// trusted — fan spin-up or an SD write burst can sag the pack momentarily —
// so it must stay low across WS_LOWBATT_CONFIRM_READS consecutive re-reads
// spaced WS_LOWBATT_CONFIRM_MS apart. Any single recovered reading cancels.
// Returns only if the battery is OK; otherwise never returns.
void checkBrownout() {
  if (vBat >= WS_LOWBATT_CUTOFF_V || vBat <= 0.5f) return;  // <=0.5 = ADC glitch, ignore

  Serial.print("vBat low ("); Serial.print(vBat, 2);
  Serial.println(" V) — confirming...");
  for (int i = 0; i < WS_LOWBATT_CONFIRM_READS; i++) {
    delay(WS_LOWBATT_CONFIRM_MS);
    readVBat();
    if (vBat >= WS_LOWBATT_CUTOFF_V) {
      Serial.println("vBat recovered — transient sag, continuing.");
      return;
    }
  }
  lowBattShutdown();   // confirmed; does not return
}

// ----------------------------------------------------------------------------
// Display
// ----------------------------------------------------------------------------
void displayWeather() {

  if (digitalRead(DISPLAY_BTN_PIN) == LOW) {
    oledSleep();
    return;
  }
  oledWake();  // re-wake if button just went high

  DateTime now = rtc.now();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print(" "); display.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  display.print("temp C :"); display.println(tempC, 2);
  display.print("dew  C :"); display.println(dewPointC, 2);
  display.print("temp F :"); display.println(tempF, 2);
  display.print("humid %:"); display.println(humidity, 2);
  display.print("hPa    :"); display.println(pressurehPa, 2);
  display.print("chip C :"); display.println(rp2040TempC, 1);
  display.print("vBat   :"); display.println(vBat, 2); // fw shuts down at 3.50V (rail collapses ~3.45)
  display.display();
}

// Read sensors + battery, append a CSV row, print to serial. No display.
// Brownout check runs AFTER the row is written: the triggering sample still
// gets logged (the rail is fine at 3.50 V — the margin exists exactly so this
// last write is safe), and the marker line lands directly after it.
void sampleLog() {
  readSensors();
  readVBat();
  readChipTemp();    // RP2040 die temp — read after vBat so the ADC ch4 leftover doesn't matter
  ensureDailyFile();
  writeRow();
  checkBrownout();   // never returns if battery is confirmed low

  Serial.print(rtc.now().timestamp(DateTime::TIMESTAMP_FULL));
  Serial.print("  T=");   Serial.print(tempC, 2);
  Serial.print("C  RH="); Serial.print(humidity, 2);
  Serial.print("%  Td="); Serial.print(dewPointC, 2);
  Serial.print("C  P=");  Serial.print(pressurehPa, 2);
  Serial.print("hPa  Tchip="); Serial.print(rp2040TempC, 1);
  Serial.print("C  V="); Serial.println(vBat, 2);
}

// MODE_SLEEP: log, wake the OLED to show the reading briefly, then sleep it.
void sampleShowSleep() {
  sampleLog();
  if (digitalRead(DISPLAY_BTN_PIN) == HIGH) {
    oledWake();
    displayWeather();
    delay(WS_DISPLAY_ON_MS);
    oledSleep();
  }
}

// MODE_CONTINUOUS: log and refresh the (always-on) OLED.
void sampleShowContinuous() {
  sampleLog();
  displayWeather();
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);   // give USB serial a moment (don't block on !Serial)

  pinMode(DISPLAY_BTN_PIN, INPUT_PULLUP); // button to turn the screen on/off

  Wire.begin();
  Wire.setClock(100000);

  analogReadResolution(12);   // required for the /4095 battery math

  // --- Display ---
  display.begin(OLED_ADDR, true);
  display.setRotation(3);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Weather Stn v2 <3");
  display.display();
  delay(1500);

  // --- RTC ---
  if (!rtc.begin()) {
    Serial.println("Could not find RTC");
    display.println("RTC FAILED");
    display.display();
    while (1) {}
  }
  rtc.writeSqwPinMode(DS3231_OFF);          // INT pin used for alarms, not square wave
  if (rtc.lostPower()) setRtcCompileTimeUTC();
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), rtcWakeISR, FALLING);
  display.println("RTC      healthy!");
  display.display();

  // --- SD (needs RTC time for the filename) ---
  setupSD();
  display.println("SD Card  healthy!");
  display.display();

  // --- Sensors ---
  setupSHT31();
  display.print("SHT31    "); display.println(sht31Ok ? "healthy!" : "MISSING");
  display.display();
  setupDPS310();
  display.print("DPS310   "); display.println(dpsOk ? "healthy!" : "MISSING");
  display.display();
  delay(5000);

  // --- Battery sanity check before committing to a deployment ---
  // Catches the "deployed with a nearly dead pack" case at power-on instead of
  // an hour into the field day.
  readVBat();
  if (vBat > 0.5f && vBat < WS_LOWBATT_CUTOFF_V) {
    Serial.println("Battery already below cutoff at boot.");
    timestamp_filename = "";   // nothing written yet; marker goes to today's file
    ensureDailyFile();
    lowBattShutdown();         // does not return
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("");
  display.println(" Sophie LV Scopazzi");
  display.println(" Small Weather Stn v2");
  display.println("");
  if (deviceMode == MODE_SLEEP) {
    display.println("   sleep mode");
    display.print("   every "); display.print(WS_WAIT_MINUTES); display.println(" min");
  } else {
    display.println("   continuous");
    display.print("   every ");
    display.print(WS_CONTINUOUS_INTERVAL_MS / 1000UL); display.println(" sec");
  }
  display.display();
  delay(4000);

  // First reading now, so a fresh deploy logs immediately and you can confirm
  // sensors + SD before anything else happens.
  if (deviceMode == MODE_SLEEP) {
    sampleShowSleep();
    armNextAlarm();              // arm the first wake
  } else {
    sampleShowContinuous();      // OLED stays on; no alarm needed
  }
}

void loop() {
  if (deviceMode == MODE_SLEEP) {
    // Halt the CPU until the RTC alarm pulls RTC_INT_PIN low.
    while (!rtcAlarmFired) {
      __wfi();
    }
    delay(10);   // wakeup stability
    sampleShowSleep();
    armNextAlarm();

  } else {  // MODE_CONTINUOUS
    static unsigned long lastSampleMs = 0;
    if (millis() - lastSampleMs >= WS_CONTINUOUS_INTERVAL_MS) {
      lastSampleMs = millis();
      sampleShowContinuous();
    }
  }
}