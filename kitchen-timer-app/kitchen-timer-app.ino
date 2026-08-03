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

// Passive buzzer / piezo -> keep 1 (uses tone()).
// Active buzzer module (makes its own tone) -> set to 0.
#define BUZZER_USE_TONE 1
const unsigned int BUZZER_HZ = 2400;

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
const unsigned long HOLD_LONG_MS    = 3000;    // program a key / cancel a set mode
const unsigned long REPEAT_DELAY_MS = 600;     // key auto-repeat kick-in
const unsigned long REPEAT_RATE_MS  = 130;     // key auto-repeat rate
const unsigned long RTC_POLL_MS     = 200;
const unsigned long RENDER_MS       = 40;
const unsigned long TIMER_MAX_SEC   = 99UL * 3600UL + 59UL * 60UL + 59UL;

const pulse_t DISPLAY_BRIGHTNESS = PULSE10_16; // PULSE1_16 (dim) .. PULSE14_16 (bright)

// ------------------------------------------------------------------ buttons
enum {
  BTN_SET     = 0,  // S1  SET       - cycle modes / hold 1s = pause / hold 3s = cancel
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
// Live values, loaded from EEPROM at boot and re-programmable by holding a key 3 s.
const unsigned int PRESET_DEFAULT[8] = { 0, 120, 60, 30, 15, 10, 7, 3 };
unsigned int presetMin[8];

const unsigned int PRESET_MAX_MIN = 99 * 60 + 59;   // 99:59 expressed in minutes

// ------------------------------------------------------------------ EEPROM map
// 0     magic
// 1     layout version
// 2..15 seven 16-bit presets for S2..S8, little endian
const int  EE_ADDR_MAGIC   = 0;
const int  EE_ADDR_VERSION = 1;
const int  EE_ADDR_PRESETS = 2;
const byte EE_MAGIC        = 0x4B;
const byte EE_VERSION      = 0x01;

// ------------------------------------------------------------------ state
// careful: this library's argument order is (clk, dio, stb)
TM1638 module(PIN_TM_CLK, PIN_TM_DIO, PIN_TM_STB);

enum Mode { MODE_NORMAL, MODE_SET_TIMER, MODE_SET_CLOCK };
Mode mode = MODE_NORMAL;

// clock
byte clkH = 0, clkM = 0, clkS = 0;
bool rtcOk = false;
unsigned long lastRtcPoll = 0;
unsigned long swTickRef   = 0;   // software clock fallback reference
byte editClkH = 0, editClkM = 0;

// countdown timer
unsigned long remainSec = 0;
bool timerRunning       = false;
unsigned long tickRef   = 0;
byte editTimH = 0, editTimM = 0;
bool wasRunning  = false;      // run state to restore if a set mode is cancelled
byte programKey  = 0xFF;       // in timer-set mode: which key's preset is being edited

// alarm
bool alarmActive = false;
unsigned long alarmStart = 0;
bool buzzerOn = false;

// button tracking
byte btnRaw = 0, btnStable = 0;
unsigned long btnChangeAt = 0;
unsigned long holdStart[8];
unsigned long nextRepeat[8];
bool longHandled[8];           // this hold already did its job; ignore the release
byte setHoldStage = 0;         // SET held in normal mode: 1 = paused, 2 = timer cleared

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

// ================================================================== debug
#if DEBUG_SERIAL
void dbg2(byte v) { if (v < 10) Serial.print('0'); Serial.print(v); }

const char *modeName() {
  switch (mode) {
    case MODE_SET_TIMER: return "SET_TIMER";
    case MODE_SET_CLOCK: return "SET_CLOCK";
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
  if (alarmActive) Serial.print(" *ALARM*");
  if (mode == MODE_SET_TIMER) {
    Serial.print(" edit="); dbg2(editTimH); Serial.print(':'); dbg2(editTimM);
    if (programKey != 0xFF) { Serial.print(" prog=S"); Serial.print(programKey + 1); }
  } else if (mode == MODE_SET_CLOCK) {
    Serial.print(" edit="); dbg2(editClkH); Serial.print(':'); dbg2(editClkM);
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
  static bool     lastRun = false, lastAlarm = false, lastRtc = false;
  static byte     lastEdit = 0xFF;
  byte editSum = (byte)(editTimH + editTimM + editClkH + editClkM);

  if ((int)mode == lastMode && remainSec == lastRem && timerRunning == lastRun &&
      alarmActive == lastAlarm && rtcOk == lastRtc && editSum == lastEdit) return;

  lastMode = (int)mode; lastRem = remainSec; lastRun = timerRunning;
  lastAlarm = alarmActive; lastRtc = rtcOk; lastEdit = editSum;
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
void buzzerSet(bool on) {
  if (on == buzzerOn) return;
  buzzerOn = on;
#if BUZZER_USE_TONE
  if (on) tone(PIN_BUZZER, BUZZER_HZ);
  else    noTone(PIN_BUZZER);
#else
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
#endif
}

// ================================================================== timer
void timerStart(unsigned long seconds) {
  if (seconds > TIMER_MAX_SEC) seconds = TIMER_MAX_SEC;
  remainSec    = seconds;
  timerRunning = (seconds > 0);
  tickRef      = millis();
}

void timerStopAndClear() {
  remainSec    = 0;
  timerRunning = false;
}

void alarmBegin() {
  timerRunning = false;
  remainSec    = 0;
  alarmActive  = true;
  alarmStart   = millis();
}

void alarmStop() {
  alarmActive = false;
  buzzerSet(false);
  timerStopAndClear();
  // always land back in normal mode; go through enterMode() when leaving the
  // clock editor so the edited time still gets written to the DS3231
  if (mode == MODE_SET_CLOCK) enterMode(MODE_NORMAL);
  else                        mode = MODE_NORMAL;
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

  bool stored = presetsLoad();
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

#if DEBUG_SERIAL
  Serial.print(stored ? F("presets from EEPROM:") : F("presets reset to defaults:"));
  for (byte i = 1; i < 8; i++) {
    Serial.print(F(" S")); Serial.print(i + 1); Serial.print('=');
    Serial.print(presetMin[i] / 60u); Serial.print(':');
    dbg2((byte)(presetMin[i] % 60u));
  }
  Serial.println();
#else
  (void)stored;
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

void updateAlarm() {
  if (!alarmActive) return;
  unsigned long el = millis() - alarmStart;
  if (el >= ALARM_LEN_MS) { alarmStop(); return; }
  buzzerSet(((el / BEEP_MS) % 2) == 0);
}

// ================================================================== actions
void enterMode(Mode next) {
  // ---- leaving the current mode (this is where edits get committed)
  if (mode == MODE_SET_CLOCK && next != MODE_SET_CLOCK) {
    if (rtcOk) {
      rtcWriteTime(editClkH, editClkM, 0);
    } else {
      clkH = editClkH; clkM = editClkM; clkS = 0;
      swTickRef = millis();
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
    } else {
      unsigned long secs = (unsigned long)editTimH * 3600UL + (unsigned long)editTimM * 60UL;
      if (secs > 0) timerStart(secs + TIMER_LEAD_SEC);  // leaving the mode starts it
      else          timerStopAndClear();
    }
  }

  // ---- entering the new mode
  if (next == MODE_SET_CLOCK) {
    editClkH = clkH;
    editClkM = clkM;
  }
  if (next == MODE_SET_TIMER) {
    wasRunning   = timerRunning;
    timerRunning = false;                              // frozen while editing
    editTimH = (byte)(remainSec / 3600UL);
    editTimM = (byte)((remainSec % 3600UL) / 60UL);
  }

  mode = next;
}

// hold a preset key for 3 s in normal mode -> edit that key's stored preset
void enterProgramMode(byte btn) {
  wasRunning   = timerRunning;
  timerRunning = false;
  programKey   = btn;
  editTimH     = (byte)(presetMin[btn] / 60u);
  editTimM     = (byte)(presetMin[btn] % 60u);
  mode         = MODE_SET_TIMER;
}

// hold SET for 3 s inside a set mode -> discard the edit, straight back to normal
void cancelSetMode() {
  if (mode == MODE_SET_TIMER) {
    programKey   = 0xFF;
    timerRunning = wasRunning;                         // countdown carries on
    tickRef      = millis();
  }
  mode = MODE_NORMAL;                                  // clock edits simply dropped
}

void onSetShort() {
  switch (mode) {
    case MODE_NORMAL:    enterMode(MODE_SET_TIMER); break;
    // when reprogramming a key, SET saves it and returns straight to normal
    case MODE_SET_TIMER: enterMode(programKey != 0xFF ? MODE_NORMAL : MODE_SET_CLOCK); break;
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
  // while the alarm rings, any key just silences and resets it
  if (alarmActive) {
    dbgKey(btn, "alarm off");
    alarmStop();
    longHandled[btn] = true;
    if (btn == BTN_SET) setHoldStage = 2;            // don't also act on this hold
    return;
  }

  if (btn == BTN_SET) { setHoldStage = 0; return; }  // SET acts on release or on hold

  // In normal mode the preset waits for the release, so that a 3 s hold can mean
  // "reprogram this key" without first firing the old preset.
  if (mode == MODE_NORMAL) return;

  if (btn >= BTN_ADJUST_FIRST && btn <= BTN_ADJUST_LAST) {
    dbgKey(btn, "adjust");
    adjustKey(btn);
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

// ================================================================== buttons
void updateButtons() {
  unsigned long now = millis();

  byte raw = module.getButtons();
  if (raw != btnRaw) { btnRaw = raw; btnChangeAt = now; }

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
        // keep holding past the pause to wipe the countdown: 1 s pause, 3 s clear
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
        longHandled[i] = true;                         // 3 s -> cancel the set mode
        onButtonHold(i);
      }
      continue;
    }

    if (mode == MODE_NORMAL) {                         // 3 s -> reprogram this key
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

void render() {
  bool blinkOn = ((millis() / BLINK_MS) % 2) == 0;

  // ---- left block: current time
  byte lh        = (mode == MODE_SET_CLOCK) ? editClkH : clkH;
  byte lm        = (mode == MODE_SET_CLOCK) ? editClkM : clkM;
  bool leftBlank = (mode == MODE_SET_CLOCK) && !blinkOn;
  bool leftDot   = (mode == MODE_SET_CLOCK) ? true : ((clkS & 1) == 0);
  drawBlock(0, lh, lm, leftBlank, leftDot);

  // ---- right block: countdown timer
  byte rh, rm;
  bool rightBlank = false, rightDot = false;
  if (mode == MODE_SET_TIMER) {
    rh = editTimH;
    rm = editTimM;
    rightBlank = !blinkOn;
    rightDot   = true;
  } else {
    rh = (byte)(remainSec / 3600UL);
    rm = (byte)((remainSec % 3600UL) / 60UL);
    if (alarmActive) { rightBlank = !blinkOn; }
    else             { rightDot   = timerRunning && blinkOn; }
  }
  drawBlock(4, rh, rm, rightBlank, rightDot);

  // ---- LEDs mirror the state (bit0 = LED1, left-most)
  byte leds = 0x00;
  if (alarmActive)                 leds = blinkOn ? 0xFF : 0x00;
  else if (mode == MODE_SET_CLOCK) leds = 0x0F;                  // left four
  else if (programKey != 0xFF)     leds = (byte)(1 << programKey); // the key being programmed
  else if (mode == MODE_SET_TIMER) leds = 0xF0;                  // right four
  else if (timerRunning)           leds = 0x80;                  // running
  else if (remainSec > 0)          leds = blinkOn ? 0x80 : 0x00; // paused
  setLeds(leds);
}

// ================================================================== loop
void loop() {
  static unsigned long lastRender = 0;

  updateButtons();
  updateClock();
  updateTimer();
  updateAlarm();

  unsigned long now = millis();
  if (now - lastRender >= RENDER_MS) {
    lastRender = now;
    render();
  }

  dbgPoll();
}
