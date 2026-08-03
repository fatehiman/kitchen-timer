# Kitchen Timer

An 8-digit kitchen timer built on an **Arduino Leonardo**, a **TM1638** module
(8× seven-segment digits + 8 LEDs + 8 buttons), a **DS3231** I²C real-time clock
and a **buzzer**.

The display is split in two halves:

```
  1 2 . 3 4     0 2 . 0 0
  └─ clock ─┘   └─ timer ─┘
   HH.MM 24h     HH.MM countdown
```

- **Left 4 digits** – current time of day, `HH.MM`, 24-hour format. The dot of the
  2nd digit turns on/off once per second as a seconds pulse.
- **Right 4 digits** – countdown timer, `HH.MM`, up to `99:59` (99 hours 59 minutes).
  Powers up at `00.00`.

Sketch: [kitchen-timer-app/kitchen-timer-app.ino](kitchen-timer-app/kitchen-timer-app.ino)

---

## 1. Wiring

All three devices run from the Leonardo's **5 V** rail and share **GND**.

### TM1638 module

| TM1638 pin | Leonardo pin | Note |
|---|---|---|
| VCC | 5V | |
| GND | GND | |
| DIO | **D8** | data, bidirectional |
| CLK | **D9** | clock, output |
| STB | **D10** | strobe, output |

Matches `TM1638 module(8, 9, 10);` — the library's argument order is
`(dataPin, clockPin, strobePin)`.

### DS3231 RTC module (I²C)

| DS3231 pin | Leonardo pin | Note |
|---|---|---|
| VCC | 5V | module has a 3.3 V regulator + level-safe pull-ups |
| GND | GND | |
| SDA | **D2 / SDA** | on the Leonardo, SDA **is** digital pin 2 |
| SCL | **D3 / SCL** | on the Leonardo, SCL **is** digital pin 3 |
| SQW | *not connected* | |
| 32K | *not connected* | |

> ⚠️ On the Leonardo, `D2` and `D3` **are** the I²C bus (unlike the Uno, where I²C
> is on A4/A5). Do not use D2/D3 for anything else. The Leonardo also breaks SDA/SCL
> out again on the two pins next to AREF — those are electrically the same pins.
>
> If your DS3231 module has a **CR2032** in it, make sure it is a rechargeable
> LIR2032 or that the charging resistor/diode is removed — most cheap ZS-042
> modules try to trickle-charge a non-rechargeable cell.

### Buzzer

| Buzzer | Leonardo pin |
|---|---|
| + / S / I-O | **D5** |
| − / GND | GND |
| VCC (3-pin active modules only) | 5V |

- **Passive buzzer / piezo** → leave `#define BUZZER_USE_TONE 1`; the sketch drives
  it with `tone()` — 2400 Hz for the alarm, 3200 Hz for the 30 ms key click.
- **Active buzzer module** (produces its own tone from DC) → set
  `#define BUZZER_USE_TONE 0`; the sketch then just switches D5 high/low.
- A loud buzzer can draw more than the 20 mA an I/O pin should source. For those,
  drive it through a small NPN transistor (e.g. BC547/2N3904): D5 → 1 kΩ → base,
  emitter → GND, buzzer between 5 V and collector, plus a 1N4148 across the buzzer
  if it is an electromagnetic type.

### Pin summary

| Leonardo pin | Used for |
|---|---|
| D2 (SDA) | DS3231 SDA |
| D3 (SCL) | DS3231 SCL |
| D5 | Buzzer |
| D8 | TM1638 DIO |
| D9 | TM1638 CLK |
| D10 | TM1638 STB |
| 5V, GND | power for all three modules |

D0/D1 (Serial1), D13 (LED) and the USB pins stay free. If you need to move the
TM1638 or buzzer, change the constants at the top of the sketch — any free digital
pin works.

---

## 2. Software setup

1. **Arduino IDE** → *Tools → Board → Arduino Leonardo*, pick the right COM port.
2. Install the TM1638 library — *Sketch → Include Library → Manage Libraries…*,
   search **TM1638** and install the one by **Damien Varrel**
   (<https://github.com/dvarrel/TM1638>), v1.0.1.
   The sketch uses `displayDig`, `writeLeds`, `getButtons`, `reset`,
   `displayTurnOn` and `displaySetBrightness` from it.
3. No RTC library is needed — the DS3231 is read/written directly over `Wire`
   (BCD registers `0x00`–`0x02`, plus the oscillator-stop flag in `0x0F`).
4. Open [kitchen-timer-app/kitchen-timer-app.ino](kitchen-timer-app/kitchen-timer-app.ino)
   and upload.

Verified build for `arduino:avr:leonardo`: **13 508 bytes flash (47 %)**,
**793 bytes RAM (30 %)**.

The whole sketch is non-blocking (`millis()`-based, no `delay()`), so buttons stay
responsive while the timer runs and the alarm sounds.

### Two gotchas about this library

The library in the Arduino Library Manager is **not** the `tm1638-library` from
Google Code / rjbatista that most tutorials (including the `setDisplayToDecNumber`
example) use. Its API differs in two ways that matter, both already handled in the
sketch:

- **Constructor order is `(CLK, DIO, STB)`**, not `(DIO, CLK, STB)`. The sketch
  passes `TM1638 module(PIN_TM_CLK, PIN_TM_DIO, PIN_TM_STB)`, so the wiring table
  above is still correct — don't "fix" it by swapping the wires.
- **`displayDig(digitId, …)` counts digits from the RIGHT** (`digitId 0` is the
  right-most digit). The sketch works in left-to-right positions and converts with
  `7 - pos` inside `setCell()`. If your module happens to be wired the other way
  and the display comes out mirrored, change that one expression to `pos`.

`displayDig()` takes a raw `pgfedcba` segment byte, where **bit 7 (`0x80`) is the
decimal point** — that is how the seconds pulse and the countdown dots are driven,
and it also lets a digit be blanked completely (`0x00`) for the blink effects.

### Serial debug output

`#define DEBUG_SERIAL 1` (default) prints state to USB serial at **115200 baud**.
One line is emitted on boot, on every keypress, and whenever the mode, timer value,
run/pause state, alarm state or RTC availability changes:

```
[B] t=0s clock=14:32:07 rtc=OK mode=NORMAL timer=00:00:00 stop
[K] S2 preset -> NORMAL
[T] t=3s clock=14:32:10 rtc=OK mode=NORMAL timer=02:00:59 RUN
[T] t=4s clock=14:32:11 rtc=OK mode=NORMAL timer=02:00:58 RUN
```

Tags: `B` = boot, `K` = key, `T` = state change. Set `DEBUG_SERIAL` to `0` to drop
it and save ~2 kB of flash. On the Leonardo `Serial` is USB CDC, and the sketch
never waits for a host to attach, so it runs standalone just fine.

---

## 2b. Command-line workflow (arduino-cli)

`arduino-cli` is installed at `%LOCALAPPDATA%\Programs\arduino-cli\arduino-cli.exe`
and is on your user PATH. One-time setup already done: `config init`,
`core install arduino:avr`, `lib install TM1638`.

```powershell
# compile
arduino-cli compile --fqbn arduino:avr:leonardo e:\www\kitchen-timer\kitchen-timer-app

# find the board
arduino-cli board list

# upload
arduino-cli upload -p COM11 --fqbn arduino:avr:leonardo e:\www\kitchen-timer\kitchen-timer-app

# serial monitor at the debug baud rate (Ctrl+C to quit)
arduino-cli monitor -p COM11 --config baudrate=115200

# compile + upload + monitor in one go
arduino-cli compile -u -p COM11 --fqbn arduino:avr:leonardo e:\www\kitchen-timer\kitchen-timer-app
```

### Leonardo upload note

Uploading to a Leonardo is a two-step dance: the tool opens the sketch's port at
**1200 baud** to reset the board, then flashes the *separate* COM port that the
Caterina bootloader exposes for ~8 seconds. On this board that bootloader port
never appears — after the 1200-baud touch the USB device goes silent for ~6 s and
then comes straight back running the old sketch, so `avrdude` has nothing to open
and fails with `cannot open port \\.\COM11`.

If you hit that, in order of likelihood:

1. **Double-tap RESET** on the board and start the upload within the 8 s window.
2. Reinstall the bootloader-port driver, or install the Arduino IDE's bundled
   drivers so `VID_2341 PID_0036` (*Arduino Leonardo bootloader*) gets a COM port.
3. If no bootloader port ever shows up, the Caterina bootloader is missing or
   damaged (typical when a board has been programmed over ISP) — reflash it with
   *Tools → Burn Bootloader* using an ISP programmer or a second Arduino.

---

## 3. Buttons

The TM1638's buttons are S1…S8, left to right.

The two hour keys sit next to each other, and so do the two minute keys.

| # | Label | In **normal** mode | In **timer set** mode | In **time set** mode |
|---|---|---|---|---|
| S1 | `SET` | short → timer set mode; **hold 1 s** → pause/resume; **hold 3 s** → clear the countdown to `00.00` | short → time set mode; **hold 3 s** → cancel | short → back to normal; **hold 3 s** → cancel |
| S2 | `h+ / 2:00` | start timer at **2:00** | hours +1 (0…99, wraps to 0) | hours +1 (0…23, wraps to 0) |
| S3 | `h- / 1:00` | start timer at **1:00** | hours −1 (wraps 0 → 99) | hours −1 (wraps 0 → 23) |
| S4 | `m+ / 0:30` | start timer at **0:30** | minutes +1 (0…59, wraps) | minutes +1 (0…59, wraps) |
| S5 | `m- / 0:15` | start timer at **0:15** | minutes −1 (wraps 0 → 59) | minutes −1 (wraps 0 → 59) |
| S6 | `0:10` | start timer at **0:10** | *(no effect)* | *(no effect)* |
| S7 | `0:07` | start timer at **0:07** | *(no effect)* | *(no effect)* |
| S8 | `0:03` | start timer at **0:03** | *(no effect)* | *(no effect)* |

**Holding any of S2…S8 for 3 s in normal mode reprograms that key's preset** — see
§3b below.

- The four `+/-` keys **auto-repeat** when held (after 600 ms, then ~8 steps/second),
  so setting 99 hours doesn't need 99 presses. Holding past 3 s keeps repeating.
- All keys are debounced (25 ms).
- Every keypress makes a **30 ms click** on the buzzer (3200 Hz, a bit higher than the
  alarm's 2400 Hz) — see §3c.
- In normal mode a preset fires **on key release**, not on press, so that a 3 s hold
  can mean "reprogram" without first launching the old preset. The delay is
  imperceptible in normal use.
- **While the alarm is ringing, any key silences it** and resets the timer to `00.00`.

### Holding `SET` in normal mode

The hold is staged, so one key covers both jobs:

| hold time | action |
|---|---|
| ~1 s | pause / resume the running countdown |
| ~3 s | **clear the countdown to `00.00`** and stop it |

Keep holding through both stages to wipe the timer — it pauses on the way past 1 s
and is then cleared at 3 s, so the end result is simply `00.00`. Release before 1 s
for the normal short press (enter *timer set* mode).

### Cancelling a set mode

**Hold `SET` for 3 s** while in *timer set* or *time set* mode to abandon the edit:
nothing is written to the RTC or to EEPROM, and you drop straight back to normal
mode. A countdown that was running when you entered the set mode resumes exactly
where it was.

Compare with a **short** `SET` press, which *commits* the edit and moves on to the
next mode.

## 3b. Reprogramming the preset keys (saved in EEPROM)

Any of the seven preset keys can be given a new duration, and it survives a power
cycle.

1. In normal mode, **hold the key for 3 s**. The right 4 digits start blinking with
   that key's current preset, and the LED above the key lights up so you know which
   one you are editing.
2. Set the new value with `h+` `h-` `m+` `m-` (S2…S5).
3. Press **`SET`** to save. The new value is written to EEPROM and you return to
   normal mode.
   Or **hold `SET` 3 s** to cancel and keep the old value.

Notes:

- Saving a preset **does not start the timer** — it is a configuration action. If a
  countdown was already running when you started programming, it resumes untouched.
- A preset may be set to anything from `00:00` to `99:59`. A key programmed to
  `00:00` simply does nothing when pressed.
- Presets are stored at EEPROM addresses 0–15: a magic byte (`0x4B`), a layout
  version byte, then seven little-endian 16-bit values in **minutes**. On boot the
  sketch validates the magic, version and range of every entry, and rewrites the
  factory defaults if anything looks wrong — so a blank or corrupted EEPROM is
  self-healing. Writes use a read-compare-write helper, so unchanged bytes don't
  burn EEPROM cycles.
- To force all keys back to their factory values, change `EE_VERSION` in the sketch
  and re-upload.

### The hidden 59 seconds

Every preset, and every timer you set by hand, is loaded with **+59 extra seconds**
(`TIMER_LEAD_SEC` in the sketch). Pressing `0:03` really loads `00:03:59`.

That is what makes the countdown honest. Because only `HH.MM` is displayed, the
seconds have to be arranged so that each displayed minute lasts a full 60 s *and*
the alarm lands the instant the display rolls `00.01 → 00.00`:

| elapsed | remaining | display |
|---|---|---|
| 0 s | `00:03:59` | `00.03` |
| 60 s | `00:02:59` | `00.02` |
| 120 s | `00:01:59` | `00.01` |
| **180 s** | `00:00:59` | `00.00` + **alarm** |

So `0:03` really is 3 whole minutes from press to buzzer. The timer fires as soon as
`remainSec` drops below 60 rather than waiting for it to reach zero — otherwise the
display would sit on `00.00` for a silent final minute.

This also makes S6 exactly "10 min and 59 sec" as originally specified.

## 3c. Key click

Pressing any of the 8 buttons fires a **tiny chirp**: `CLICK_MS` = 30 ms at
`CLICK_HZ` = 3200 Hz. Details worth knowing:

- It is triggered **on key-down only**, right after debounce. Holding a key for 3 s
  still makes a single 30 ms sound, not a 3 s tone — the chirp is one-shot and
  non-blocking (`updateClick()` ends it from `loop()`, like everything else here).
- **Auto-repeat does not click.** Holding `h+` steps ~8×/second, which would turn
  into a machine-gun beep.
- The pitch is deliberately **higher than the alarm** (3200 vs 2400 Hz) so the two are
  never confused. On an **active** buzzer (`BUZZER_USE_TONE 0`) pitch isn't ours to
  pick, so the click is just a 30 ms blip of whatever tone the module makes.
- **While the alarm is ringing, keys do not click.** That press exists to silence the
  buzzer; chirping at the same moment would be noise. If the alarm happens to start
  *during* a click, the alarm takes the buzzer over immediately.
- Set `CLICK_MS` to `0` to turn the click off completely.

---

## 4. Modes

`SET` cycles: **normal → timer set → time set → normal**.

| Mode | Left 4 digits (clock) | Right 4 digits (timer) |
|---|---|---|
| **normal** | steady, dot pulses each second | steady; dots blink 1 Hz while counting down |
| **timer set** | steady, dot pulses | **blinks** (500 ms on / 500 ms off) |
| **time set** | **blinks** | steady |

- Entering **timer set** freezes the countdown. Leaving it (with `SET`) starts the
  timer at the new value if it is greater than `00.00`, or clears it if it is `00.00`.
- Entering **time set** copies the current time into an edit buffer; leaving it
  writes `HH:MM:00` to the DS3231 and clears the RTC's oscillator-stop flag.
- In normal mode nothing blinks except the clock's seconds dot.
- *Timer set* mode is also where preset reprogramming happens (§3b); the LED pattern
  tells the two apart.
- Holding `SET` for 3 s leaves any set mode **without** saving.

---

## 5. Countdown and alarm behaviour

1. Timer starts at `00.00` on power-up.
2. While running, the remaining time is displayed as **floor** hours/minutes: at
   `1:59:59` the display shows `1:59`.
3. While running, the dot between the timer's `HH` and `MM` blinks once per second.
4. The alarm fires **the moment the display changes from `00.01` to `00.00`**, so the
   full set duration has genuinely elapsed and there is no silent final minute:
   - all **4 timer digits blink** (500 / 500 ms),
   - the buzzer beeps **500 ms on / 500 ms off**,
   - both continue for **1 minute**.
5. After that minute the alarm stops on its own, the timer resets to `00.00` and
   stops blinking.
   **Pressing any of the 8 buttons ends the alarm early** with the same result: buzzer
   off, timer cleared to `00.00`, back in normal mode — and that press does nothing
   else, so it cannot accidentally start a new countdown. If the alarm happened to go
   off while you were in *time set* mode, the edited time is still saved to the RTC on
   the way out.
6. Maximum settable value is `99:59` (`99 h 59 m`).

## 6. LED indicators

The 8 LEDs above the digits echo the state, which is handy while debugging:

| State | LEDs |
|---|---|
| Alarm ringing | all 8 blink |
| Time set mode | left 4 on |
| Timer set mode | right 4 on |
| Reprogramming a preset key | only the LED above that key |
| Timer running | LED 8 on |
| Timer paused (long-press `SET`) | LED 8 blinks |
| Idle | all off |

## 7. Robustness

- If the DS3231 does not answer (not wired yet, bad solder joint, dead bus), the
  sketch falls back to a **software clock** driven by `millis()` so the timer half
  keeps working. As soon as the RTC answers again, its time takes over.
- The DS3231 is polled 5×/second; the display is refreshed 25×/second and only the
  digits that actually changed are re-sent to the TM1638, so there is no flicker.
- 12-hour-mode RTC registers are converted to 24 h on read, in case the chip was
  previously configured that way.

## 8. Tunable constants

At the top of the sketch:

| Constant | Default | Meaning |
|---|---|---|
| `TIMER_LEAD_SEC` | `59` | extra seconds added to every preset / set value |
| `ALARM_LEN_MS` | `60000` | how long the alarm rings |
| `BEEP_MS` | `500` | alarm buzzer on/off period |
| `CLICK_MS` | `30` | key-down click length, `0` = no click |
| `CLICK_HZ` | `3200` | key-down click pitch (passive buzzer only) |
| `BLINK_MS` | `500` | digit blink half-period |
| `HOLD_MS` | `1000` | `SET` hold in normal mode = pause/resume |
| `HOLD_LONG_MS` | `3000` | hold to reprogram a key / cancel a set mode |
| `PRESET_DEFAULT` | `120,60,30,15,10,7,3` | factory presets in minutes for S2…S8 |
| `REPEAT_DELAY_MS` / `REPEAT_RATE_MS` | `600` / `130` | key auto-repeat |
| `DISPLAY_BRIGHTNESS` | `PULSE10_16` | brightness, `PULSE1_16` (dim) … `PULSE14_16` |
| `BUZZER_USE_TONE` | `1` | 1 = passive buzzer via `tone()`, 0 = active buzzer |
| `DEBUG_SERIAL` | `1` | 1 = serial debug at 115200, 0 = silent |

## 9. Reference

- TM1638 library actually used: <https://github.com/dvarrel/TM1638>
- The tutorial library (different API, not used here):
  <https://github.com/rjbatista/tm1638-library>
  (original, now archived: <https://code.google.com/p/tm1638-library/>)
- TM1638 datasheet (English): <https://arduinolearning.s3.amazonaws.com/TM1638English%20version.pdf>
- Modules: <http://s.click.aliexpress.com/e/3ZbAemeie> ·
  <http://s.click.aliexpress.com/e/Q3FyNr76m>
