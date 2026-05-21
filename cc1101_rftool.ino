/*
 * CC1101 Universal 433 MHz RF Tool  |  ESP32-WROOM-32
 *
 * Wiring (8-pin CC1101 module):
 *   GND-GND  VCC-3V3 (1.8-3.6V only!)  CSN-GPIO5  SCK-GPIO18  MOSI-GPIO23  MISO-GPIO19
 *   GDO0-GPIO2 (TX data)               GDO2-GPIO4 (RX data)
 *
 * Libraries: "SmartRC-CC1101-Driver-Lib" (LSatan), "rc-switch" (sui77)
 * Serial @ 115200, line ending = Newline. Type ? for commands.
 *
 * Modes: 'decode' lets rc-switch identify known protocols; 'raw' captures the
 * bare pulse timing of any OOK burst so it can be replayed like a copy fob.
 * All settings persist in NVS flash.
 */

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <RCSwitch.h>
#include <Preferences.h>

#define PIN_SCK 18
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_CSN 5
#define PIN_GDO0 2
#define PIN_GDO2 4

#define RAW_MAX 512        // max edges per capture
#define RAW_GAP_US 3000    // gap longer than this starts a new frame
#define RAW_IDLE_US 60000  // line quiet this long means the burst ended
#define RAW_MIN 16         // shorter captures are treated as noise

#define STARTUP_DELAY_MS 3000  // gives you time to open the serial monitor

RCSwitch mySwitch = RCSwitch();
Preferences prefs;

struct Config {
  float freq;
  int mod;                   // 0=2FSK 1=GFSK 2=ASK/OOK 3=4FSK 4=MSK
  float rxbw;                // kHz
  int pa;                    // dBm
  unsigned int pulse;        // us
  unsigned int repeat;       // coded-send repeats / raw replay repeats
  unsigned long durationMs;  // total coded-send time
  int txProto;
  unsigned int txBits;
  int filterProto;  // -1 = show all
  int filterBits;   // 0 = any
} cfg;

enum Mode { MODE_DECODE,
            MODE_RAW };
Mode mode = MODE_DECODE;

volatile unsigned int rawBuf[RAW_MAX];
volatile unsigned int rawIdx = 0;
volatile unsigned long lastEdgeMicros = 0;
volatile bool rawStartLevel = true;

unsigned int capBuf[RAW_MAX];
unsigned int capLen = 0;
bool capStartLevel = true;
bool rxAttached = false;

// Records the time between every edge on GDO2. A long gap is read as a frame
// boundary, so each new frame overwrites the last and we keep the cleanest one.
// IRAM_ATTR is required for ISRs on the ESP32.
void IRAM_ATTR rawISR() {
  unsigned long now = micros();
  unsigned long dur = now - lastEdgeMicros;
  lastEdgeMicros = now;
  if (dur > RAW_GAP_US) {
    rawIdx = 0;
    rawStartLevel = (digitalRead(PIN_GDO2) != 0);
    return;
  }
  if (rawIdx < RAW_MAX) rawBuf[rawIdx++] = (unsigned int)dur;
}

// ---------------------------------------------------------------------------
// Settings persistence (NVS flash)
void loadConfig() {
  prefs.begin("rftool", true);
  cfg.freq = prefs.getFloat("freq", 433.92);
  cfg.mod = prefs.getInt("mod", 2);
  cfg.rxbw = prefs.getFloat("rxbw", 135.0);
  cfg.pa = prefs.getInt("pa", 10);
  cfg.pulse = prefs.getUInt("pulse", 500);
  cfg.repeat = prefs.getUInt("repeat", 8);
  cfg.durationMs = prefs.getULong("dur", 5000);
  cfg.txProto = prefs.getInt("txproto", 11);
  cfg.txBits = prefs.getUInt("txbits", 12);
  cfg.filterProto = prefs.getInt("fproto", -1);
  cfg.filterBits = prefs.getInt("fbits", 0);
  prefs.end();
}

void saveConfig() {
  prefs.begin("rftool", false);
  prefs.putFloat("freq", cfg.freq);
  prefs.putInt("mod", cfg.mod);
  prefs.putFloat("rxbw", cfg.rxbw);
  prefs.putInt("pa", cfg.pa);
  prefs.putUInt("pulse", cfg.pulse);
  prefs.putUInt("repeat", cfg.repeat);
  prefs.putULong("dur", cfg.durationMs);
  prefs.putInt("txproto", cfg.txProto);
  prefs.putUInt("txbits", cfg.txBits);
  prefs.putInt("fproto", cfg.filterProto);
  prefs.putInt("fbits", cfg.filterBits);
  prefs.end();
}

void defaultConfig() {
  cfg = { 433.92, 2, 135.0, 10, 500, 8, 5000, 11, 12, -1, 0 };
}

// ---------------------------------------------------------------------------
// Radio control
void radioInit() {
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(cfg.freq);
  ELECHOUSE_cc1101.setModulation(cfg.mod);
  ELECHOUSE_cc1101.setRxBW(cfg.rxbw);
  ELECHOUSE_cc1101.setPA(cfg.pa);
}

// rxAttached guards against detaching an interrupt that was never attached,
// which otherwise prints "GPIO number error" on the ESP32.
void detachAll() {
  if (!rxAttached) return;
  mySwitch.disableReceive();
  detachInterrupt(PIN_GDO2);
  rxAttached = false;
}

void enterRx() {
  ELECHOUSE_cc1101.SetRx();
  if (mode == MODE_DECODE) {
    mySwitch.enableReceive(PIN_GDO2);
    mySwitch.resetAvailable();
  } else {
    rawIdx = 0;
    lastEdgeMicros = micros();
    attachInterrupt(PIN_GDO2, rawISR, CHANGE);
  }
  rxAttached = true;
}

void applyRadioAndListen() {
  detachAll();
  radioInit();
  enterRx();
}

// ---------------------------------------------------------------------------
// Transmit

// Re-initialising the radio after every transmit is essential: on the ESP32
// the CC1101 otherwise stays stuck in TX and never receives again until a
// power cycle. radioInit() reproduces the clean power-on state.
void sendCode(unsigned long code, unsigned int bits) {
  detachAll();
  ELECHOUSE_cc1101.SetTx();
  mySwitch.setProtocol(cfg.txProto);
  mySwitch.setPulseLength(cfg.pulse);
  mySwitch.setRepeatTransmit(cfg.repeat);

  Serial.print("TX code=");
  Serial.print(code);
  Serial.print(" bits=");
  Serial.print(bits);
  Serial.print(" proto=");
  Serial.print(cfg.txProto);
  Serial.print(" pulse=");
  Serial.print(cfg.pulse);
  Serial.print(" for ");
  Serial.print(cfg.durationMs / 1000.0, 1);
  Serial.println("s ...");

  unsigned long start = millis();
  while (millis() - start < cfg.durationMs) {
    mySwitch.send(code, bits);
    yield();  // keep the watchdog fed during long sends
  }
  radioInit();
  enterRx();
  Serial.println("TX done, listening again.");
}

// Bit-bangs the captured edge timings back out on GDO0, alternating the line
// level for each stored duration. Protocol-agnostic, so it copies signals that
// rc-switch can't decode.
void replayRaw() {
  if (capLen < RAW_MIN) {
    Serial.println("No raw signal captured. Switch to 'raw' and trigger the remote first.");
    return;
  }
  detachAll();
  ELECHOUSE_cc1101.SetTx();
  Serial.print("Replaying ");
  Serial.print(capLen);
  Serial.print(" edges x ");
  Serial.print(cfg.repeat);
  Serial.println(" ...");

  for (unsigned int r = 0; r < cfg.repeat; r++) {
    bool level = capStartLevel;
    for (unsigned int i = 0; i < capLen; i++) {
      digitalWrite(PIN_GDO0, level ? HIGH : LOW);
      delayMicroseconds(capBuf[i]);
      level = !level;
    }
    digitalWrite(PIN_GDO0, LOW);
    delay(15);
    yield();
  }
  radioInit();
  enterRx();
  Serial.println("Replay done, listening again.");
}

// ---------------------------------------------------------------------------
// Helpers
const char* modName(int m) {
  switch (m) {
    case 0: return "2-FSK";
    case 1: return "GFSK";
    case 2: return "ASK/OOK";
    case 3: return "4-FSK";
    case 4: return "MSK";
    default: return "?";
  }
}

bool isNumeric(const String& s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++)
    if (!isDigit(s.charAt(i))) return false;
  return true;
}

void printSettings() {
  Serial.println(F("---- Settings ----"));
  Serial.print(F("mode        "));
  Serial.println(mode == MODE_DECODE ? "decode" : "raw");
  Serial.print(F("freq        "));
  Serial.print(cfg.freq, 2);
  Serial.println(" MHz");
  Serial.print(F("modulation  "));
  Serial.print(cfg.mod);
  Serial.print(" (");
  Serial.print(modName(cfg.mod));
  Serial.println(")");
  Serial.print(F("rxbw        "));
  Serial.print(cfg.rxbw, 2);
  Serial.println(" kHz");
  Serial.print(F("pa          "));
  Serial.print(cfg.pa);
  Serial.println(" dBm");
  Serial.print(F("pulse       "));
  Serial.print(cfg.pulse);
  Serial.println(" us");
  Serial.print(F("repeat      "));
  Serial.println(cfg.repeat);
  Serial.print(F("duration    "));
  Serial.print(cfg.durationMs / 1000.0, 1);
  Serial.println(" s");
  Serial.print(F("txproto     "));
  Serial.println(cfg.txProto);
  Serial.print(F("txbits      "));
  Serial.println(cfg.txBits);
  Serial.print(F("filter      "));
  if (cfg.filterProto < 0) Serial.print("all");
  else Serial.print(cfg.filterProto);
  Serial.print(F("  bits "));
  if (cfg.filterBits == 0) Serial.println("any");
  else Serial.println(cfg.filterBits);
  Serial.print(F("captured    "));
  Serial.print(capLen);
  Serial.println(" edges");
}

void printHelp() {
  Serial.println(F("==== CC1101 Universal RF Tool ===="));
  Serial.println(F("Listening:"));
  Serial.println(F("  decode            rc-switch protocol decoding (default)"));
  Serial.println(F("  raw               capture raw OOK timing of any burst"));
  Serial.println(F("Transmit:"));
  Serial.println(F("  <code> [bits]     send a code, e.g. 1996  or  1996 24"));
  Serial.println(F("  send <code> [bits]same as above"));
  Serial.println(F("  replay            replay the last raw capture"));
  Serial.println(F("  rawdump           print the last captured timings"));
  Serial.println(F("Diagnostics:"));
  Serial.println(F("  rssi              live signal strength at current freq"));
  Serial.println(F("  scan              sweep 433.0-434.5 MHz, report strongest"));
  Serial.println(F("Radio settings (saved to flash):"));
  Serial.println(F("  freq <MHz>        e.g. freq 433.92  (300-348/387-464/779-928)"));
  Serial.println(F("  mod <ook|2fsk|gfsk|4fsk|msk>"));
  Serial.println(F("  rxbw <kHz>        58..812"));
  Serial.println(F("  pa <dBm>          -30..12"));
  Serial.println(F("  pulse <us>        rc-switch send pulse length"));
  Serial.println(F("  repeat <n>        send/replay repeats"));
  Serial.println(F("  duration <s>      coded-send total time"));
  Serial.println(F("  txproto <n>       rc-switch send protocol"));
  Serial.println(F("  bits <n>          default send bit length"));
  Serial.println(F("  filter <all|n>    decode protocol filter"));
  Serial.println(F("  filterbits <any|n>decode bit-length filter"));
  Serial.println(F("System:"));
  Serial.println(F("  settings / ?      show settings & this help"));
  Serial.println(F("  save / defaults   persist / restore default settings"));
}

// ---------------------------------------------------------------------------
// Command parser
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  String low = cmd;
  low.toLowerCase();

  int sp = low.indexOf(' ');
  String first = (sp == -1) ? low : low.substring(0, sp);
  String rest = (sp == -1) ? "" : low.substring(sp + 1);
  rest.trim();

  if (first == "?" || first == "help" || first == "settings") {
    printHelp();
    printSettings();
    return;
  }

  if (first == "decode") {
    mode = MODE_DECODE;
    applyRadioAndListen();
    Serial.println("Mode: decode");
    return;
  }
  if (first == "raw" || first == "sniff") {
    mode = MODE_RAW;
    capLen = 0;
    applyRadioAndListen();
    Serial.println("Mode: raw (trigger the remote now)");
    return;
  }

  if (first == "replay") {
    replayRaw();
    return;
  }
  if (first == "rawdump") {
    if (capLen < RAW_MIN) {
      Serial.println("Nothing captured.");
      return;
    }
    Serial.print("startLevel=");
    Serial.print(capStartLevel ? "HIGH" : "LOW");
    Serial.print("  edges=");
    Serial.println(capLen);
    for (unsigned int i = 0; i < capLen; i++) {
      Serial.print(capBuf[i]);
      Serial.print(i + 1 < capLen ? "," : "\n");
    }
    return;
  }

  // Live RSSI. Numbers near the noise floor (~-78 dBm) mean nothing is there;
  // a real signal lifts well above it. Hold the fob a meter away to avoid
  // overloading the receiver, which smears energy across the whole band.
  if (first == "rssi") {
    detachAll();
    ELECHOUSE_cc1101.SetRx();
    delay(5);
    Serial.print("RSSI monitor @ ");
    Serial.print(cfg.freq, 2);
    Serial.println(" MHz - hold the fob button. Send any line to stop.");
    while (!Serial.available()) {
      Serial.print("RSSI ");
      Serial.print(ELECHOUSE_cc1101.getRssi());
      Serial.println(" dBm");
      delay(150);
    }
    while (Serial.available()) Serial.read();
    applyRadioAndListen();
    return;
  }

  // Sweeps the band to find a transmitter's true centre frequency, which can
  // sit noticeably off the nominal 433.92. Hold the fob button the whole time.
  if (first == "scan") {
    detachAll();
    Serial.println("Scanning 433.0-434.5 MHz - hold the fob button down the whole time...");
    float best = 0;
    int bestR = -200;
    for (float f = 433.00; f <= 434.50; f += 0.05) {
      ELECHOUSE_cc1101.setMHZ(f);
      ELECHOUSE_cc1101.SetRx();
      int peak = -200;
      for (int s = 0; s < 6; s++) {
        delay(8);
        int r = ELECHOUSE_cc1101.getRssi();
        if (r > peak) peak = r;
      }
      Serial.print(f, 2);
      Serial.print(" MHz  ");
      Serial.print(peak);
      Serial.println(" dBm");
      if (peak > bestR) {
        bestR = peak;
        best = f;
      }
      yield();
    }
    Serial.print("Peak: ");
    Serial.print(best, 2);
    Serial.print(" MHz @ ");
    Serial.print(bestR);
    Serial.println(" dBm");
    applyRadioAndListen();
    return;
  }

  // Radio settings: changing any of these needs a full radio re-init to apply.
  if (first == "freq") {
    float v = rest.toFloat();
    if (v >= 300 && v <= 928) {
      cfg.freq = v;
      saveConfig();
      applyRadioAndListen();
      Serial.print("freq = ");
      Serial.print(v, 2);
      Serial.println(" MHz");
    } else Serial.println("freq out of range (300-928 MHz)");
    return;
  }
  if (first == "mod") {
    int m = -1;
    if (rest == "2fsk") m = 0;
    else if (rest == "gfsk") m = 1;
    else if (rest == "ook" || rest == "ask") m = 2;
    else if (rest == "4fsk") m = 3;
    else if (rest == "msk") m = 4;
    else if (isNumeric(rest)) m = rest.toInt();
    if (m >= 0 && m <= 4) {
      cfg.mod = m;
      saveConfig();
      applyRadioAndListen();
      Serial.print("mod = ");
      Serial.println(modName(m));
    } else Serial.println("mod: use ook|2fsk|gfsk|4fsk|msk");
    return;
  }
  if (first == "rxbw") {
    float v = rest.toFloat();
    if (v >= 58 && v <= 812) {
      cfg.rxbw = v;
      saveConfig();
      applyRadioAndListen();
      Serial.print("rxbw = ");
      Serial.print(v, 2);
      Serial.println(" kHz");
    } else Serial.println("rxbw out of range (58-812 kHz)");
    return;
  }
  if (first == "pa") {
    int v = rest.toInt();
    if (v >= -30 && v <= 12) {
      cfg.pa = v;
      saveConfig();
      applyRadioAndListen();
      Serial.print("pa = ");
      Serial.print(v);
      Serial.println(" dBm");
    } else Serial.println("pa out of range (-30..12 dBm)");
    return;
  }

  // Send/replay settings: applied at the next transmit, no radio re-init needed.
  if (first == "pulse") {
    if (isNumeric(rest) && rest.toInt() > 0) {
      cfg.pulse = rest.toInt();
      saveConfig();
      Serial.print("pulse = ");
      Serial.println(cfg.pulse);
    } else Serial.println("usage: pulse <us>");
    return;
  }
  if (first == "repeat") {
    if (isNumeric(rest) && rest.toInt() > 0) {
      cfg.repeat = rest.toInt();
      saveConfig();
      Serial.print("repeat = ");
      Serial.println(cfg.repeat);
    } else Serial.println("usage: repeat <n>");
    return;
  }
  if (first == "duration") {
    if (rest.toFloat() > 0) {
      cfg.durationMs = (unsigned long)(rest.toFloat() * 1000);
      saveConfig();
      Serial.print("duration = ");
      Serial.print(cfg.durationMs / 1000.0, 1);
      Serial.println(" s");
    } else Serial.println("usage: duration <seconds>");
    return;
  }
  if (first == "txproto") {
    if (isNumeric(rest)) {
      cfg.txProto = rest.toInt();
      saveConfig();
      Serial.print("txproto = ");
      Serial.println(cfg.txProto);
    } else Serial.println("usage: txproto <n>");
    return;
  }
  if (first == "bits") {
    if (isNumeric(rest) && rest.toInt() >= 1 && rest.toInt() <= 32) {
      cfg.txBits = rest.toInt();
      saveConfig();
      Serial.print("bits = ");
      Serial.println(cfg.txBits);
    } else Serial.println("usage: bits <1..32>");
    return;
  }

  if (first == "filter") {
    if (rest == "all") cfg.filterProto = -1;
    else if (isNumeric(rest)) cfg.filterProto = rest.toInt();
    else {
      Serial.println("usage: filter <all|n>");
      return;
    }
    saveConfig();
    Serial.println("filter updated");
    return;
  }
  if (first == "filterbits") {
    if (rest == "any") cfg.filterBits = 0;
    else if (isNumeric(rest)) cfg.filterBits = rest.toInt();
    else {
      Serial.println("usage: filterbits <any|n>");
      return;
    }
    saveConfig();
    Serial.println("filterbits updated");
    return;
  }

  if (first == "save") {
    saveConfig();
    Serial.println("Settings saved.");
    return;
  }
  if (first == "defaults") {
    defaultConfig();
    saveConfig();
    applyRadioAndListen();
    Serial.println("Defaults restored.");
    return;
  }

  if (first == "send") {
    int s2 = rest.indexOf(' ');
    String c = (s2 == -1) ? rest : rest.substring(0, s2);
    String b = (s2 == -1) ? "" : rest.substring(s2 + 1);
    b.trim();
    if (isNumeric(c)) {
      unsigned int bits = cfg.txBits;
      if (isNumeric(b) && b.toInt() >= 1 && b.toInt() <= 32) bits = b.toInt();
      sendCode(strtoul(c.c_str(), NULL, 10), bits);
    } else Serial.println("usage: send <code> [bits]");
    return;
  }

  // A bare number is shorthand for "send <number>".
  if (isNumeric(first)) {
    unsigned int bits = cfg.txBits;
    if (isNumeric(rest) && rest.toInt() >= 1 && rest.toInt() <= 32) bits = rest.toInt();
    sendCode(strtoul(first.c_str(), NULL, 10), bits);
    return;
  }

  Serial.print("Unknown command: '");
  Serial.print(cmd);
  Serial.println("'  (type ? for help)");
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(STARTUP_DELAY_MS);

  pinMode(PIN_GDO0, OUTPUT);
  digitalWrite(PIN_GDO0, LOW);

  loadConfig();

  // SPI and GDO pins must be set before Init().
  ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  ELECHOUSE_cc1101.setGDO(PIN_GDO0, PIN_GDO2);
  radioInit();

  if (ELECHOUSE_cc1101.getCC1101()) Serial.println("CC1101: connection OK");
  else Serial.println("CC1101: connection ERROR - check wiring/3V3 power!");

  mySwitch.enableTransmit(PIN_GDO0);
  enterRx();

  Serial.println();
  printHelp();
  printSettings();
  Serial.println("\nReady.");
}

void loop() {
  if (mode == MODE_DECODE && mySwitch.available()) {
    unsigned long value = mySwitch.getReceivedValue();
    int proto = mySwitch.getReceivedProtocol();
    int bits = mySwitch.getReceivedBitlength();
    bool okP = (cfg.filterProto < 0) || (proto == cfg.filterProto);
    bool okB = (cfg.filterBits == 0) || (bits == cfg.filterBits);
    if (okP && okB) {
      Serial.print("RX code=");
      Serial.print(value);
      Serial.print(" proto=");
      Serial.print(proto);
      Serial.print(" bits=");
      Serial.print(bits);
      Serial.print(" pulse=");
      Serial.println(mySwitch.getReceivedDelay());
    }
    mySwitch.resetAvailable();
  }

  // Finalise a raw capture once the line has been quiet long enough, copying
  // the volatile ISR buffer into a stable one with interrupts briefly off.
  if (mode == MODE_RAW && rawIdx >= RAW_MIN && (micros() - lastEdgeMicros) > RAW_IDLE_US) {
    noInterrupts();
    capLen = rawIdx;
    if (capLen > RAW_MAX) capLen = RAW_MAX;
    for (unsigned int i = 0; i < capLen; i++) capBuf[i] = rawBuf[i];
    capStartLevel = rawStartLevel;
    rawIdx = 0;
    interrupts();
    Serial.print("RAW captured ");
    Serial.print(capLen);
    Serial.print(" edges (start ");
    Serial.print(capStartLevel ? "HIGH" : "LOW");
    Serial.println(").  Type 'replay' to transmit it, 'rawdump' to view.");
  }

  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
}
