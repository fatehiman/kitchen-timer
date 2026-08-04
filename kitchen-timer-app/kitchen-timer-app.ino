/*
 * Kitchen Timer - Arduino Leonardo + TM1638 + DS3231 + buzzer
 *
 * Left 4 digits : current time HH.MM (24h), dot of the 2nd digit pulses once per second
 * Right 4 digits: countdown timer HH.MM (max 99:59), dots blink while counting down
 *
 * See ../README.md for wiring and the full behaviour description.
 *
 * Library: "TM1638" by Damien Varrel  (Arduino Library Manager -> search TM1638)
 *          https://github.com/dvarrel/TM1638
 *          NOTE its constructor order is (CLK, DIO, STB) and its digitId 0 is the
 *          RIGHT-most digit - both are handled below.
 * The DS3231 is driven directly over Wire, so no RTC library is needed.
 */

#include <Wire.h>
#include <EEPROM.h>
#include <TM1638.h>

// ------------------------------------------------------------------ pin map
const byte PIN_TM_DIO = 8;   // TM1638 DIO
const byte PIN_TM_CLK = 9;   // TM1638 CLK
const byte PIN_TM_STB = 10;  // TM1638 STB
const byte PIN_BUZZER = 5;   // buzzer (+) or transistor base resistor
const byte PIN_TOUCH  = 4;   // TTP223 touch pad, active HIGH

// Passive buzzer / piezo -> keep 1 (uses tone()).
// Active buzzer module (makes its own tone) -> set to 0.
#define BUZZER_USE_TONE 1
const unsigned int BUZZER_HZ = 2400;

// Key-down click: a short chirp, deliberately a different pitch from the alarm.
// Set CLICK_MS to 0 to disable the click entirely.
const unsigned int  CLICK_HZ = 3200;
const unsigned long CLICK_MS = 30;

const byte DS3231_ADDR = 0x68;

// Set to 0 to drop all serial debug output (saves ~2 kB of flash).
#define DEBUG_SERIAL 1
const unsigned long DEBUG_BAUD = 115200;

// ------------------------------------------------------------------ tuning
// Seconds added on top of every HH:MM the user asks for. 59 makes the displayed
// HH.MM tick down a full minute at a time and puts the alarm exactly at the moment
// the display rolls 00.01 -> 00.00, i.e. "0:03" really is 3 whole minutes.
const unsigned long TIMER_LEAD_SEC = 59;
const unsigned long ALARM_LEN_MS    = 60000UL; // alarm rings for 1 minute
const unsigned long BEEP_MS         = 500;     // 500 ms on / 500 ms off
const unsigned long BLINK_MS        = 500;     // blink half-period
const unsigned long DEBOUNCE_MS     = 25;
const unsigned long HOLD_MS         = 1000;    // SET hold in normal mode = pause
const unsigned long HOLD_LONG_MS    = 2000;    // program a key / cancel a set mode
const unsigned long REPEAT_DELAY_MS = 600;     // key auto-repeat kick-in
const unsigned long REPEAT_RATE_MS  = 130;     // key auto-repeat rate
const unsigned long SET_IDLE_MS     = 60000UL;  // no key for 1 min in a set mode -> cancel
const unsigned long RTC_POLL_MS     = 200;
const unsigned long RENDER_MS       = 40;
const unsigned long TIMER_MAX_SEC   = 99UL * 3600UL + 59UL * 60UL + 59UL;

const pulse_t DISPLAY_BRIGHTNESS = PULSE10_16; // PULSE1_16 (dim) .. PULSE14_16 (bright)

// ------------------------------------------------------------------ buttons
enum {
  BTN_SET     = 0,  // S1  SET       - cycle modes / hold 1s = pause / hold 2s = cancel
  BTN_H_PLUS  = 1,  // S2  h+  2:00
  BTN_H_MINUS = 2,  // S3  h-  1:00
  BTN_M_PLUS  = 3,  // S4  m+  0:30
  BTN_M_MINUS = 4,  // S5  m-  0:15
  BTN_Q10     = 5,  // S6  0:10
  BTN_Q07     = 6,  // S7  0:07
  BTN_Q03     = 7   // S8  0:03
};
const byte BTN_ADJUST_FIRST = BTN_H_PLUS;    // S2..S5 are the four +/- keys
const byte BTN_ADJUST_LAST  = BTN_M_MINUS;

// Preset, in minutes, that each button loads in normal mode (index 0 = SET, unused).
// Live values, loaded from EEPROM at boot and re-programmable by holding a key 2 s.
const unsigned int PRESET_DEFAULT[8] = { 0, 120, 60, 30, 15, 10, 7, 3 };
unsigned int presetMin[8];

const unsigned int PRESET_MAX_MIN = 99 * 60 + 59;   // 99:59 expressed in minutes

// ------------------------------------------------------------------ touch pad
// One tap loads the next duration in this ring, in minutes. 0 = clear the timer,
// and the ring then starts over at 3 min. Every value gets TIMER_LEAD_SEC on top,
// exactly like a preset key, so "3" really is 3 whole minutes.
const unsigned int TOUCH_SEQ[] = { 3, 5, 7, 10, 15, 30, 45, 60, 90, 120, 150, 180, 0 };
const byte TOUCH_SEQ_N = sizeof(TOUCH_SEQ) / sizeof(TOUCH_SEQ[0]);
byte touchIdx = 0;                 // where the next tap picks up

// ------------------------------------------------------------------ time alarms
const byte AL_COUNT = 3;
byte alH[AL_COUNT], alM[AL_COUNT];
bool alOn[AL_COUNT];
unsigned int alFired[AL_COUNT];    // minute-of-day this alarm last fired at

// ------------------------------------------------------------------ EEPROM map
// 0     magic
// 1     layout version
// 2..15 seven 16-bit presets for S2..S8, little endian
// 16    alarm-block magic          } kept as a separate block with its own magic so
// 17    alarm-block version        } adding it does not wipe the stored presets
// 18..26 three alarms: hh, mm, armed
const int  EE_ADDR_MAGIC   = 0;
const int  EE_ADDR_VERSION = 1;
const int  EE_ADDR_PRESETS = 2;
const byte EE_MAGIC        = 0x4B;
const byte EE_VERSION      = 0x01;

const int  EE_ADDR_AL_MAGIC   = 16;
const int  EE_ADDR_AL_VERSION = 17;
const int  EE_ADDR_ALARMS     = 18;
const byte EE_AL_MAGIC        = 0x41;
const byte EE_AL_VERSION      = 0x01;

// ------------------------------------------------------------------ state
// careful: this library's argument order is (clk, dio, stb)
TM1638 module(PIN_TM_CLK, PIN_TM_DIO, PIN_TM_STB);

// SET cycles: normal -> timer -> clock -> alarm1 -> alarm2 -> alarm3 -> normal.
// The three alarm modes are kept adjacent and last so (mode - MODE_SET_AL1) is the
// alarm index everywhere below.
enum Mode { MODE_NORMAL, MODE_SET_TIMER, MODE_SET_CLOCK,
            MODE_SET_AL1, MODE_SET_AL2, MODE_SET_AL3 };
Mode mode = MODE_NORMAL;

static bool isAlarmMode(Mode m) { return m >= MODE_SET_AL1; }
static byte alarmModeIdx()      { return (byte)(mode - MODE_SET_AL1); }

// clock
byte clkH = 0, clkM = 0, clkS = 0;
bool rtcOk = false;
unsigned long lastRtcPoll = 0;
unsigned long swTickRef   = 0;   // software clock fallback reference
byte editClkH = 0, editClkM = 0;
byte origClkH = 0, origClkM = 0;   // value the editor started from, frozen while editing

// countdown timer
unsigned long remainSec = 0;
bool timerRunning       = false;
unsigned long tickRef   = 0;
byte editTimH = 0, editTimM = 0;
byte origTimH = 0, origTimM = 0;   // ditto, so an untouched editor can be a no-op
bool wasRunning  = false;      // run state to restore if a set mode is cancelled
byte programKey  = 0xFF;       // in timer-set mode: which key's preset is being edited

// alarm edit buffers (shared by the three set-alarm modes)
byte editAlH = 0, editAlM = 0;
bool editAlOn = false;
byte origAlH = 0, origAlM = 0;
bool origAlOn = false;

// ringing state: the countdown alarm and the three time alarms are independent,
// so several can be sounding at once
bool timerAlarm = false;
unsigned long timerAlarmStart = 0;
byte tAlarmRing = 0;                        // bit i = time alarm i is ringing
unsigned long tAlarmStart[AL_COUNT];

bool buzzerOn = false;
unsigned int buzzerHz = 0;      // pitch currently being driven

static bool ringing() { return timerAlarm || tAlarmRing != 0; }

// key click
bool clickActive = false;
unsigned long clickStart = 0;

// touch pad tracking
bool touchRaw = false, touchStable = false;
unsigned long touchChangeAt = 0;

// button tracking
byte btnRaw = 0, btnStable = 0;
unsigned long btnChangeAt = 0;
unsigned long holdStart[8];
unsigned long nextRepeat[8];
bool longHandled[8];           // this hold already did its job; ignore the release
byte setHoldStage = 0;         // SET held in normal mode: 1 = paused, 2 = timer cleared
unsigned long setActivityAt = 0;   // last key activity, for the set-mode idle timeout

// display cache: raw segment byte per digit, so only changed digits are re-sent.
// Index 0 = LEFT-most digit (the library's own numbering is reversed).
byte shownSeg[8];
byte shownLeds = 0xFF;                 // impossible value -> forces a first write

// segment patterns, bit order pgfedcba; bit7 (0x80) is the decimal point
const byte SEG_DIGIT[10] = {
  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};
const byte SEG_BLANK = 0x00;
const byte SEG_DOT   = 0x80;
const byte SEG_A     = 0x77;   // "AL-n", shown while a time alarm rings
const byte SEG_L     = 0x38;
const byte SEG_DASH  = 0x40;

// ================================================================== EEPROM
static void eepUpdate(int addr, byte v) {
  if (EEPROM.read(addr) != v) EEPROM.write(addr, v);   // spare the write cycles
}

static void eepWriteWord(int addr, unsigned int v) {
  eepUpdate(addr, (byte)(v & 0xFF));
  eepUpdate(addr + 1, (byte)(v >> 8));
}

static unsigned int eepReadWord(int addr) {
  return (unsigned int)EEPROM.read(addr) | ((unsigned int)EEPROM.read(addr + 1) << 8);
}

void presetsSaveAll() {
  eepUpdate(EE_ADDR_MAGIC, EE_MAGIC);
  eepUpdate(EE_ADDR_VERSION, EE_VERSION);
  for (byte i = 1; i < 8; i++) {
    eepWriteWord(EE_ADDR_PRESETS + (i - 1) * 2, presetMin[i]);
  }
}

void presetSaveOne(byte btn) {
  eepUpdate(EE_ADDR_MAGIC, EE_MAGIC);
  eepUpdate(EE_ADDR_VERSION, EE_VERSION);
  eepWriteWord(EE_ADDR_PRESETS + (btn - 1) * 2, presetMin[btn]);
}

// returns true when stored values were used, false when defaults were written
bool presetsLoad() {
  for (byte i = 0; i < 8; i++) presetMin[i] = PRESET_DEFAULT[i];

  if (EEPROM.read(EE_ADDR_MAGIC) != EE_MAGIC ||
      EEPROM.read(EE_ADDR_VERSION) != EE_VERSION) {
    presetsSaveAll();                       // first boot, or a layout change
    return false;
  }

  for (byte i = 1; i < 8; i++) {
    unsigned int v = eepReadWord(EE_ADDR_PRESETS + (i - 1) * 2);
    if (v <= PRESET_MAX_MIN) presetMin[i] = v;   // otherwise keep the default
  }
  return true;
}

void alarmSaveOne(byte i) {
  eepUpdate(EE_ADDR_AL_MAGIC, EE_AL_MAGIC);
  eepUpdate(EE_ADDR_AL_VERSION, EE_AL_VERSION);
  eepUpdate(EE_ADDR_ALARMS + i * 3,     alH[i]);
  eepUpdate(EE_ADDR_ALARMS + i * 3 + 1, alM[i]);
  eepUpdate(EE_ADDR_ALARMS + i * 3 + 2, alOn[i] ? 1 : 0);
}

// returns true when stored alarms were used, false when the block was initialised
bool alarmsLoad() {
  for (byte i = 0; i < AL_COUNT; i++) { alH[i] = 0; alM[i] = 0; alOn[i] = false; }

  if (EEPROM.read(EE_ADDR_AL_MAGIC) != EE_AL_MAGIC ||
      EEPROM.read(EE_ADDR_AL_VERSION) != EE_AL_VERSION) {
    for (byte i = 0; i < AL_COUNT; i++) alarmSaveOne(i);   // first boot after upgrade
    return false;
  }

  for (byte i = 0; i < AL_COUNT; i++) {
    byte h = EEPROM.read(EE_ADDR_ALARMS + i * 3);
    byte m = EEPROM.read(EE_ADDR_ALARMS + i * 3 + 1);
    byte on = EEPROM.read(EE_ADDR_ALARMS + i * 3 + 2);
    if (h > 23 || m > 59) continue;                        // corrupt -> leave it off
    alH[i] = h; alM[i] = m; alOn[i] = (on == 1);
  }
  return true;
}

// ================================================================== debug
#if DEBUG_SERIAL
void dbg2(byte v) { if (v < 10) Serial.print('0'); Serial.print(v); }

const char *modeName() {
  switch (mode) {
    case MODE_SET_TIMER: return "SET_TIMER";
    case MODE_SET_CLOCK: return "SET_CLOCK";
    case MODE_SET_AL1:   return "SET_AL1";
    case MODE_SET_AL2:   return "SET_AL2";
    case MODE_SET_AL3:   return "SET_AL3";
    default:             return "NORMAL";
  }
}

// tag: B=boot K=key T=state change
void dbgLine(char tag) {
  Serial.print('['); Serial.print(tag); Serial.print("] t=");
  Serial.print(millis() / 1000); Serial.print('s');
  Serial.print(" clock="); dbg2(clkH); Serial.print(':'); dbg2(clkM);
  Serial.print(':'); dbg2(clkS);
  Serial.print(rtcOk ? " rtc=OK" : " rtc=--");
  Serial.print(" mode="); Serial.print(modeName());
  Serial.print(" timer=");
  dbg2((byte)(remainSec / 3600UL)); Serial.print(':');
  dbg2((byte)((remainSec % 3600UL) / 60UL)); Serial.print(':');
  dbg2((byte)(remainSec % 60UL));
  Serial.print(timerRunning ? " RUN" : " stop");
  if (timerAlarm) Serial.print(" *ALARM*");
  for (byte i = 0; i < AL_COUNT; i++) {
    if (tAlarmRing & (1 << i)) { Serial.print(" *AL-"); Serial.print(i + 1); Serial.print('*'); }
  }
  if (mode == MODE_SET_TIMER) {
    Serial.print(" edit="); dbg2(editTimH); Serial.print(':'); dbg2(editTimM);
    if (programKey != 0xFF) { Serial.print(" prog=S"); Serial.print(programKey + 1); }
  } else if (mode == MODE_SET_CLOCK) {
    Serial.print(" edit="); dbg2(editClkH); Serial.print(':'); dbg2(editClkM);
  } else if (isAlarmMode(mode)) {
    Serial.print(" edit="); dbg2(editAlH); Serial.print(':'); dbg2(editAlM);
    Serial.print(editAlOn ? " armed" : " off");
  }
  Serial.println();
}

void dbgKey(byte btn, const char *what) {
  Serial.print("[K] S"); Serial.print(btn + 1);
  Serial.print(' '); Serial.print(what); Serial.print(" -> ");
  Serial.println(modeName());
}

// print one line whenever anything user-visible changes
void dbgPoll() {
  static int      lastMode  = -1;
  static unsigned long lastRem = 0xFFFFFFFFUL;
  static bool     lastRun = false, lastRtc = false;
  static byte     lastAlarm = 0xFF, lastEdit = 0xFF;
  byte editSum   = (byte)(editTimH + editTimM + editClkH + editClkM + editAlH + editAlM);
  byte alarmBits = (byte)((timerAlarm ? 1 : 0) | (tAlarmRing << 1));

  if ((int)mode == lastMode && remainSec == lastRem && timerRunning == lastRun &&
      alarmBits == lastAlarm && rtcOk == lastRtc && editSum == lastEdit) return;

  lastMode = (int)mode; lastRem = remainSec; lastRun = timerRunning;
  lastAlarm = alarmBits; lastRtc = rtcOk; lastEdit = editSum;
  dbgLine('T');
}
#else
#define dbgLine(tag)      do {} while (0)
#define dbgKey(btn, what) do {} while (0)
#define dbgPoll()         do {} while (0)
#endif

// ================================================================== DS3231
static byte bcd2dec(byte v) { return (byte)((v >> 4) * 10 + (v & 0x0F)); }
static byte dec2bcd(byte v) { return (byte)(((v / 10) << 4) | (v % 10)); }

bool rtcReadTime() {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write((byte)0x00);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(DS3231_ADDR, (byte)3) != 3) return false;

  byte s  = bcd2dec(Wire.read() & 0x7F);
  byte m  = bcd2dec(Wire.read() & 0x7F);
  byte hr = Wire.read();
  byte h;
  if (hr & 0x40) {                       // 12h mode -> convert to 24h
    h = bcd2dec(hr & 0x1F);
    if (hr & 0x20) { if (h != 12) h += 12; }
    else           { if (h == 12) h = 0;  }
  } else {
    h = bcd2dec(hr & 0x3F);
  }
  if (s > 59 || m > 59 || h > 23) return false;

  clkH = h; clkM = m; clkS = s;
  return true;
}

void rtcWriteTime(byte h, byte m, byte s) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write((byte)0x00);
  Wire.write(dec2bcd(s));
  Wire.write(dec2bcd(m));
  Wire.write(dec2bcd(h));                // bit6 left at 0 -> 24 hour mode
  Wire.endTransmission();

  // clear the oscillator-stop flag, the time is valid again
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write((byte)0x0F);
  if (Wire.endTransmission() == 0 && Wire.requestFrom(DS3231_ADDR, (byte)1) == 1) {
    byte st = Wire.read() & 0x7F;
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write((byte)0x0F);
    Wire.write(st);
    Wire.endTransmission();
  }

  clkH = h; clkM = m; clkS = s;
  swTickRef = millis();
}

// ================================================================== buzzer
// hz is ignored for an active buzzer; no default argument, the Arduino
// prototype generator does not get along with those.
void buzzerSet(bool on, unsigned int hz) {
  if (on == buzzerOn && (!on || hz == buzzerHz)) return;
  buzzerOn = on;
  buzzerHz = hz;
#if BUZZER_USE_TONE
  if (on) tone(PIN_BUZZER, hz);
  else    noTone(PIN_BUZZER);
#else
  (void)hz;                              // active buzzer: pitch is its own business
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
#endif
}

// One tiny chirp on key-down. Non-blocking: updateClick() ends it CLICK_MS later,
// so holding a key for 2 s still only makes a 30 ms sound.
void clickBeep() {
  if (CLICK_MS == 0) return;
  if (ringing()) return;                 // that press is there to silence the alarm
  clickActive = true;
  clickStart  = millis();
  buzzerSet(true, CLICK_HZ);
}

void updateClick() {
  if (!clickActive) return;
  if (ringing()) { clickActive = false; return; }     // alarm took the buzzer over
  if (millis() - clickStart < CLICK_MS) return;
  clickActive = false;
  buzzerSet(false, CLICK_HZ);
}

// ================================================================== timer
// Anything that sets the countdown other than a touch tap invalidates the tap
// sequence, so the next tap starts over at 3 min. touchTap() re-applies its own
// index after calling these.
void timerStart(unsigned long seconds) {
  if (seconds > TIMER_MAX_SEC) seconds = TIMER_MAX_SEC;
  remainSec    = seconds;
  timerRunning = (seconds > 0);
  tickRef      = millis();
  touchIdx     = 0;
}

void timerStopAndClear() {
  remainSec    = 0;
  timerRunning = false;
  touchIdx     = 0;
}

void alarmBegin() {
  timerRunning    = false;
  remainSec       = 0;
  touchIdx        = 0;
  timerAlarm      = true;
  timerAlarmStart = millis();
}

// one tap = load the next duration in the ring and start it
void touchTap() {
  byte idx = touchIdx;
  unsigned int mins = TOUCH_SEQ[idx];
  if (mins > 0) timerStart((unsigned long)mins * 60UL + TIMER_LEAD_SEC);
  else          timerStopAndClear();
  touchIdx = (byte)((idx + 1) % TOUCH_SEQ_N);   // after the call, which resets it
}

// any key or tap silences everything that is currently sounding
void alarmStop() {
  bool wasTimerAlarm = timerAlarm;
  timerAlarm  = false;
  tAlarmRing  = 0;
  clickActive = false;
  buzzerSet(false, BUZZER_HZ);
  // a time alarm must not wipe a countdown that is still running underneath it
  if (wasTimerAlarm) timerStopAndClear();
  // always land back in normal mode; go through enterMode() so an edit in progress
  // (the clock, an alarm) still gets committed on the way out
  if (mode != MODE_NORMAL) enterMode(MODE_NORMAL);
}

// ================================================================== setup
void setup() {
#if !BUZZER_USE_TONE
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
#endif
#if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);            // Leonardo: never wait for the host here
#endif

  pinMode(PIN_TOUCH, INPUT);           // TTP223 drives the line push-pull

  bool stored = presetsLoad();
  bool alStored = alarmsLoad();
  for (byte i = 0; i < 8; i++) longHandled[i] = false;

  Wire.begin();
  module.reset();                       // blanks digits + LEDs, turns the display on
  module.displayTurnOn();
  module.displaySetBrightness(DISPLAY_BRIGHTNESS);

  for (byte i = 0; i < 8; i++) shownSeg[i] = 0xFF;   // force a full first refresh
  setLeds(0x00);

  rtcOk       = rtcReadTime();
  swTickRef   = millis();
  lastRtcPoll = millis();
  tickRef     = millis();

  // pretend every alarm already fired this minute, so booting inside an armed
  // alarm's minute does not set it off immediately
  for (byte i = 0; i < AL_COUNT; i++) alFired[i] = (unsigned int)clkH * 60u + clkM;

#if DEBUG_SERIAL
  Serial.print(stored ? F("presets from EEPROM:") : F("presets reset to defaults:"));
  for (byte i = 1; i < 8; i++) {
    Serial.print(F(" S")); Serial.print(i + 1); Serial.print('=');
    Serial.print(presetMin[i] / 60u); Serial.print(':');
    dbg2((byte)(presetMin[i] % 60u));
  }
  Serial.println();
  Serial.print(alStored ? F("alarms from EEPROM:") : F("alarms initialised:"));
  for (byte i = 0; i < AL_COUNT; i++) {
    Serial.print(F(" AL-")); Serial.print(i + 1); Serial.print('=');
    dbg2(alH[i]); Serial.print(':'); dbg2(alM[i]);
    Serial.print(alOn[i] ? F("(on)") : F("(off)"));
  }
  Serial.println();
#else
  (void)stored; (void)alStored;
#endif
  dbgLine('B');                         // boot
}

// ================================================================== clock
void softwareClockAdvance() {
  if (++clkS < 60) return;
  clkS = 0;
  if (++clkM < 60) return;
  clkM = 0;
  if (++clkH > 23) clkH = 0;
}

void updateClock() {
  unsigned long now = millis();

  if (now - lastRtcPoll >= RTC_POLL_MS) {
    lastRtcPoll = now;
    bool ok = rtcReadTime();            // refreshes clkH/clkM/clkS when it works
    if (ok) swTickRef = now;            // keep the fallback in step with the RTC
    rtcOk = ok;
  }

  if (!rtcOk) {                         // no RTC answering -> run a software clock
    while (now - swTickRef >= 1000) {
      swTickRef += 1000;
      softwareClockAdvance();
    }
  }
}

void updateTimer() {
  if (!timerRunning) return;
  unsigned long now = millis();
  while (now - tickRef >= 1000) {
    tickRef += 1000;
    if (remainSec > 0) remainSec--;
    // ring as soon as the displayed HH.MM would roll from 00.01 to 00.00,
    // not a minute later when the seconds finally run out
    if (remainSec < 60) { alarmBegin(); return; }
  }
}

// a time alarm goes off the moment the clock enters its minute, at most once per
// minute-of-day so silencing it does not let it start again a fraction later
void checkTimeAlarms() {
  unsigned int nowMin = (unsigned int)clkH * 60u + clkM;
  for (byte i = 0; i < AL_COUNT; i++) {
    if (!alOn[i]) continue;
    if (alH[i] != clkH || alM[i] != clkM) continue;
    if (alFired[i] == nowMin) continue;
    alFired[i]     = nowMin;
    tAlarmRing    |= (byte)(1 << i);
    tAlarmStart[i] = millis();
  }
}

void updateAlarm() {
  unsigned long now = millis();

  // each ringing source times out on its own after ALARM_LEN_MS
  if (timerAlarm && now - timerAlarmStart >= ALARM_LEN_MS) {
    timerAlarm = false;
    timerStopAndClear();
  }
  for (byte i = 0; i < AL_COUNT; i++) {
    if ((tAlarmRing & (1 << i)) && now - tAlarmStart[i] >= ALARM_LEN_MS) {
      tAlarmRing &= (byte)~(1 << i);
    }
  }

  if (!ringing()) {
    if (buzzerOn && !clickActive) buzzerSet(false, BUZZER_HZ);
    return;
  }
  // one shared beep pattern however many alarms are sounding, in step with the blink
  buzzerSet(((now / BEEP_MS) % 2) == 0, BUZZER_HZ);
}

// ================================================================== actions
void enterMode(Mode next) {
  // ---- leaving the current mode (this is where edits get committed)
  if (mode == MODE_SET_CLOCK && next != MODE_SET_CLOCK) {
    // Only write the RTC when the user actually turned the time. Otherwise merely
    // walking through this mode would zero the seconds every time, so the clock
    // would keep losing up to 59 s per visit.
    if (editClkH != origClkH || editClkM != origClkM) {
      if (rtcOk) {
        rtcWriteTime(editClkH, editClkM, 0);
      } else {
        clkH = editClkH; clkM = editClkM; clkS = 0;
        swTickRef = millis();
      }
    }
  }
  if (mode == MODE_SET_TIMER && next != MODE_SET_TIMER) {
    if (programKey != 0xFF) {
      // reprogramming a key: store its new preset, leave the countdown alone
      presetMin[programKey] = (unsigned int)editTimH * 60u + editTimM;
      presetSaveOne(programKey);
      programKey   = 0xFF;
      timerRunning = wasRunning;
      tickRef      = millis();
    } else if (editTimH == origTimH && editTimM == origTimM) {
      // untouched editor -> same no-op as a cancel: keep the seconds we still had
      // and the run/pause state we came in with, instead of restarting at HH:MM:59
      timerRunning = wasRunning;
      tickRef      = millis();
    } else {
      unsigned long secs = (unsigned long)editTimH * 3600UL + (unsigned long)editTimM * 60UL;
      if (secs > 0) timerStart(secs + TIMER_LEAD_SEC);  // leaving the mode starts it
      else          timerStopAndClear();
    }
  }

  if (isAlarmMode(mode) && next != mode) {
    byte i = alarmModeIdx();
    if (editAlH != origAlH || editAlM != origAlM || editAlOn != origAlOn) {
      alH[i] = editAlH; alM[i] = editAlM; alOn[i] = editAlOn;
      alarmSaveOne(i);
      // an alarm just set to the minute we are standing in waits for tomorrow
      alFired[i] = (unsigned int)clkH * 60u + clkM;
    }
  }

  // ---- entering the new mode
  if (next == MODE_SET_CLOCK) {
    editClkH = origClkH = clkH;      // both frozen from here on, however long the edit takes
    editClkM = origClkM = clkM;
  }
  if (next == MODE_SET_TIMER) {
    wasRunning   = timerRunning;
    timerRunning = false;                              // frozen while editing
    editTimH = origTimH = (byte)(remainSec / 3600UL);
    editTimM = origTimM = (byte)((remainSec % 3600UL) / 60UL);
  }
  if (isAlarmMode(next) && next != mode) {
    byte i = (byte)(next - MODE_SET_AL1);
    editAlH = origAlH = alH[i];
    editAlM = origAlM = alM[i];
    editAlOn = origAlOn = alOn[i];
  }
  if (next != MODE_NORMAL) setActivityAt = millis();    // start the idle timeout

  mode = next;
}

// hold a preset key for 2 s in normal mode -> edit that key's stored preset
void enterProgramMode(byte btn) {
  wasRunning   = timerRunning;
  timerRunning = false;
  programKey   = btn;
  editTimH = origTimH = (byte)(presetMin[btn] / 60u);
  editTimM = origTimM = (byte)(presetMin[btn] % 60u);
  setActivityAt = millis();
  mode         = MODE_SET_TIMER;
}

// hold SET for 2 s inside a set mode -> discard the edit, straight back to normal
void cancelSetMode() {
  if (mode == MODE_SET_TIMER) {
    programKey   = 0xFF;
    timerRunning = wasRunning;                         // countdown carries on
    tickRef      = millis();
  }
  mode = MODE_NORMAL;                                  // clock edits simply dropped
}

// left in a set mode with no key touched for SET_IDLE_MS -> abandon the edit
void updateSetTimeout() {
  if (mode == MODE_NORMAL) return;
  if (ringing()) return;                               // the alarm owns the exit path
  if (millis() - setActivityAt < SET_IDLE_MS) return;
#if DEBUG_SERIAL
  Serial.println(F("[K] idle timeout -> NORMAL"));
#endif
  cancelSetMode();
}

void onSetShort() {
  switch (mode) {
    case MODE_NORMAL:    enterMode(MODE_SET_TIMER); break;
    // when reprogramming a key, SET saves it and returns straight to normal
    case MODE_SET_TIMER: enterMode(programKey != 0xFF ? MODE_NORMAL : MODE_SET_CLOCK); break;
    case MODE_SET_CLOCK: enterMode(MODE_SET_AL1);   break;
    case MODE_SET_AL1:   enterMode(MODE_SET_AL2);   break;
    case MODE_SET_AL2:   enterMode(MODE_SET_AL3);   break;
    default:             enterMode(MODE_NORMAL);    break;
  }
}

void onSetLong() {
  if (mode != MODE_NORMAL) return;
  if (remainSec == 0) return;
  timerRunning = !timerRunning;                        // pause / resume
  if (timerRunning) tickRef = millis();
}

void adjustHour(int delta) {
  if (mode == MODE_SET_TIMER) {
    int h = (int)editTimH + delta;
    if (h > 99) h = 0;
    if (h < 0)  h = 99;
    editTimH = (byte)h;
  } else if (mode == MODE_SET_CLOCK) {
    int h = (int)editClkH + delta;
    if (h > 23) h = 0;
    if (h < 0)  h = 23;
    editClkH = (byte)h;
  } else if (isAlarmMode(mode)) {
    int h = (int)editAlH + delta;
    if (h > 23) h = 0;
    if (h < 0)  h = 23;
    editAlH = (byte)h;
    editAlOn = true;                 // touching the value arms the alarm
  }
}

void adjustMinute(int delta) {
  if (mode == MODE_SET_TIMER) {
    int m = (int)editTimM + delta;
    if (m > 59) m = 0;
    if (m < 0)  m = 59;
    editTimM = (byte)m;
  } else if (mode == MODE_SET_CLOCK) {
    int m = (int)editClkM + delta;
    if (m > 59) m = 0;
    if (m < 0)  m = 59;
    editClkM = (byte)m;
  } else if (isAlarmMode(mode)) {
    int m = (int)editAlM + delta;
    if (m > 59) m = 0;
    if (m < 0)  m = 59;
    editAlM = (byte)m;
    editAlOn = true;
  }
}

void adjustKey(byte btn) {
  switch (btn) {
    case BTN_H_PLUS:  adjustHour(+1);   break;
    case BTN_M_PLUS:  adjustMinute(+1); break;
    case BTN_H_MINUS: adjustHour(-1);   break;
    case BTN_M_MINUS: adjustMinute(-1); break;
  }
}

void applyPreset(byte btn) {
  unsigned int mins = presetMin[btn];
  if (mins == 0) return;
  timerStart((unsigned long)mins * 60UL + TIMER_LEAD_SEC);
}

// key pushed down
void onButtonPress(byte btn) {
  // while anything rings, any key just silences it
  if (ringing()) {
    dbgKey(btn, "alarm off");
    alarmStop();
    longHandled[btn] = true;
    if (btn == BTN_SET) setHoldStage = 2;            // don't also act on this hold
    return;
  }

  if (btn == BTN_SET) { setHoldStage = 0; return; }  // SET acts on release or on hold

  // In normal mode the preset waits for the release, so that a 2 s hold can mean
  // "reprogram this key" without first firing the old preset.
  if (mode == MODE_NORMAL) return;

  if (btn >= BTN_ADJUST_FIRST && btn <= BTN_ADJUST_LAST) {
    dbgKey(btn, "adjust");
    adjustKey(btn);
  } else if (btn == BTN_Q03 && isAlarmMode(mode)) {
    editAlOn = !editAlOn;                            // S8 arms / disarms this alarm
    dbgKey(btn, editAlOn ? "arm" : "disarm");
  } else {
    dbgKey(btn, "ignored");                          // S6..S8 are dead in the set modes
  }
}

// key released without its hold action having fired
void onButtonRelease(byte btn) {
  if (btn == BTN_SET) { onSetShort(); return; }
  if (mode != MODE_NORMAL) return;
  dbgKey(btn, "preset");
  applyPreset(btn);
}

// key held past HOLD_LONG_MS
void onButtonHold(byte btn) {
  if (btn == BTN_SET) {
    if (mode != MODE_NORMAL) { dbgKey(btn, "cancel"); cancelSetMode(); }
    return;
  }
  if (mode != MODE_NORMAL) return;
  dbgKey(btn, "program");
  enterProgramMode(btn);
}

void onButtonRepeat(byte btn) {
  if (mode == MODE_NORMAL) return;                   // presets never auto-repeat
  if (btn < BTN_ADJUST_FIRST || btn > BTN_ADJUST_LAST) return;
  adjustKey(btn);
}

// ================================================================== touch pad
void onTouchDown() {
  if (ringing()) {                       // same as any key: a tap just shuts it up
#if DEBUG_SERIAL
    Serial.println(F("[K] TOUCH alarm off"));
#endif
    alarmStop();
    return;
  }
  clickBeep();
  if (mode != MODE_NORMAL) return;       // the tap ring only makes sense in normal mode
#if DEBUG_SERIAL
  Serial.print(F("[K] TOUCH ")); Serial.print(TOUCH_SEQ[touchIdx]); Serial.println(F(" min"));
#endif
  touchTap();
}

void updateTouch() {
  unsigned long now = millis();
  bool raw = (digitalRead(PIN_TOUCH) == HIGH);

  if (raw != touchRaw) { touchRaw = raw; touchChangeAt = now; }
  if (touchStable) setActivityAt = now;

  if (touchRaw != touchStable && (now - touchChangeAt) >= DEBOUNCE_MS) {
    touchStable = touchRaw;
    if (touchStable) onTouchDown();      // acts on touch, nothing on release
  }
}

// ================================================================== buttons
void updateButtons() {
  unsigned long now = millis();

  byte raw = module.getButtons();
  if (raw != btnRaw) { btnRaw = raw; btnChangeAt = now; }

  if (btnStable) setActivityAt = now;   // any key down or held counts as activity

  if (btnRaw != btnStable && (now - btnChangeAt) >= DEBOUNCE_MS) {
    byte changed = (byte)(btnStable ^ btnRaw);
    btnStable = btnRaw;

    for (byte i = 0; i < 8; i++) {
      if (!(changed & (1 << i))) continue;
      bool down = (btnStable & (1 << i)) != 0;
      if (down) {
        holdStart[i]    = now;
        nextRepeat[i]   = now + REPEAT_DELAY_MS;
        longHandled[i]  = false;
        clickBeep();                                   // one chirp per key-down only
        onButtonPress(i);
      } else if (!longHandled[i]) {
        onButtonRelease(i);
      }
    }
  }

  for (byte i = 0; i < 8; i++) {
    if (!(btnStable & (1 << i))) continue;
    unsigned long held = now - holdStart[i];

    if (i == BTN_SET) {                                // SET never auto-repeats
      if (mode == MODE_NORMAL) {
        // keep holding past the pause to wipe the countdown: 1 s pause, 2 s clear
        if (setHoldStage < 1 && held >= HOLD_MS) {
          setHoldStage   = 1;
          longHandled[i] = true;
          dbgKey(i, timerRunning ? "pause" : "resume");
          onSetLong();
        }
        if (setHoldStage < 2 && held >= HOLD_LONG_MS) {
          setHoldStage   = 2;
          longHandled[i] = true;
          dbgKey(i, "clear timer");
          timerStopAndClear();
        }
      } else if (!longHandled[i] && held >= HOLD_LONG_MS) {
        longHandled[i] = true;                         // 2 s -> cancel the set mode
        onButtonHold(i);
      }
      continue;
    }

    if (mode == MODE_NORMAL) {                         // 2 s -> reprogram this key
      if (!longHandled[i] && held >= HOLD_LONG_MS) {
        longHandled[i] = true;
        onButtonHold(i);
      }
      continue;                                        // presets never auto-repeat
    }

    // a set mode is active: auto-repeat the four +/- keys for as long as they are held
    if (longHandled[i]) continue;
    if ((long)(now - nextRepeat[i]) < 0) continue;
    nextRepeat[i] = now + REPEAT_RATE_MS;
    onButtonRepeat(i);
  }
}

// ================================================================== display
// pos 0..7 counted from the LEFT; digit 0..9, or -1 for a blank digit
void setCell(byte pos, int8_t digit, bool dot) {
  byte seg = (digit < 0) ? SEG_BLANK : SEG_DIGIT[digit];
  if (dot) seg |= SEG_DOT;
  if (shownSeg[pos] == seg) return;
  shownSeg[pos] = seg;
  module.displayDig((byte)(7 - pos), seg);      // library counts from the right
}

// same cache, but for cells that are not a decimal digit (letters, dashes)
void setCellRaw(byte pos, byte seg) {
  if (shownSeg[pos] == seg) return;
  shownSeg[pos] = seg;
  module.displayDig((byte)(7 - pos), seg);
}

void setLeds(byte mask) {
  if (mask == shownLeds) return;
  shownLeds = mask;
  module.writeLeds(mask);
}

// draw HH.MM into the 4 cells starting at `first`
void drawBlock(byte first, byte hh, byte mm, bool blank, bool dot) {
  int8_t d[4];
  d[0] = (int8_t)(hh / 10);              // hours are always 2 digits: 01.00, 09.25
  d[1] = (int8_t)(hh % 10);
  d[2] = (int8_t)(mm / 10);
  d[3] = (int8_t)(mm % 10);
  for (byte i = 0; i < 4; i++) {
    bool cellDot = (i == 1) && dot && !blank;           // dot lives on the 2nd digit
    setCell((byte)(first + i), blank ? (int8_t)-1 : d[i], cellDot);
  }
}

// "AL-n" across the right 4 digits, blanked on the off half of the blink
void drawAlarmLabel(byte idx, bool blank) {
  if (blank) {
    for (byte i = 4; i < 8; i++) setCellRaw(i, SEG_BLANK);
    return;
  }
  setCellRaw(4, SEG_A);
  setCellRaw(5, SEG_L);
  setCellRaw(6, SEG_DASH);
  setCellRaw(7, SEG_DIGIT[idx + 1]);
}

// "--.--", the disarmed state of a time alarm in its set mode
void drawDashes(bool blank) {
  for (byte i = 0; i < 4; i++) {
    setCellRaw((byte)(4 + i), blank ? SEG_BLANK : (byte)(SEG_DASH | (i == 1 ? SEG_DOT : 0)));
  }
}

// lowest-numbered time alarm currently ringing; duplicates simply share its label
byte lowestRinging() {
  for (byte i = 0; i < AL_COUNT; i++) if (tAlarmRing & (1 << i)) return i;
  return 0;
}

void render() {
  bool blinkOn = ((millis() / BLINK_MS) % 2) == 0;

  // ---- left block: current time
  byte lh        = (mode == MODE_SET_CLOCK) ? editClkH : clkH;
  byte lm        = (mode == MODE_SET_CLOCK) ? editClkM : clkM;
  bool leftBlank = (mode == MODE_SET_CLOCK) && !blinkOn;
  bool leftDot   = (mode == MODE_SET_CLOCK) ? true : ((clkS & 1) == 0);
  drawBlock(0, lh, lm, leftBlank, leftDot);

  // ---- right block. The countdown alarm outranks a time alarm, so when both go
  // off together the digits show a blinking 00.00 and only the LEDs say AL-n too.
  if (timerAlarm) {
    drawBlock(4, 0, 0, !blinkOn, false);
  } else if (tAlarmRing) {
    drawAlarmLabel(lowestRinging(), !blinkOn);
  } else if (mode == MODE_SET_TIMER) {
    drawBlock(4, editTimH, editTimM, !blinkOn, true);
  } else if (isAlarmMode(mode)) {
    if (editAlOn) drawBlock(4, editAlH, editAlM, !blinkOn, true);
    else          drawDashes(!blinkOn);
  } else {
    byte rh = (byte)(remainSec / 3600UL);
    byte rm = (byte)((remainSec % 3600UL) / 60UL);
    drawBlock(4, rh, rm, false, timerRunning && blinkOn);
  }

  // ---- LEDs (bit0 = LED1, left-most).
  // Right four are the four alarm channels: LED5 = countdown, LED6..8 = AL-1..3.
  // Left LEDs 2..4 stay lit for each armed time alarm.
  byte leds = 0x00;
  if (ringing()) {
    if (blinkOn) {
      if (timerAlarm) leds |= 0x10;
      leds |= (byte)(tAlarmRing << 5);
    }
  } else if (mode == MODE_SET_CLOCK) {
    leds = 0x01;                                       // LED1, the clock channel
  } else if (programKey != 0xFF) {
    leds = (byte)(1 << programKey);                    // the key being programmed
  } else if (mode == MODE_SET_TIMER) {
    leds = 0x10;                                       // LED5, the countdown channel
  } else if (isAlarmMode(mode)) {
    leds = (byte)(1 << (5 + alarmModeIdx()));          // LED6/7/8 = AL-1/2/3
  } else if (timerRunning) {
    leds = 0x10;                                       // running
  } else if (remainSec > 0) {
    leds = blinkOn ? 0x10 : 0x00;                      // paused
  }

  // armed-alarm indicators ride along in normal mode (including while ringing)
  if (mode == MODE_NORMAL) {
    for (byte i = 0; i < AL_COUNT; i++) if (alOn[i]) leds |= (byte)(1 << (1 + i));
  }
  setLeds(leds);
}

// ================================================================== loop
void loop() {
  static unsigned long lastRender = 0;

  updateButtons();
  updateTouch();
  updateSetTimeout();
  updateClock();
  checkTimeAlarms();
  updateTimer();
  updateAlarm();
  updateClick();

  unsigned long now = millis();
  if (now - lastRender >= RENDER_MS) {
    lastRender = now;
    render();
  }

  dbgPoll();
}
