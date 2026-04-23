// ============================================================================
// SERIAL CLI — interactive single-line command interface over USB-CDC
// ============================================================================
//
// Designed to replace runtime-reflashing for parameter tweaks. All globals
// and helpers from D_O_ESP32_S3.ino are visible here because the Arduino
// toolchain concatenates all .ino files in the sketch folder.
//
// Command style: flat single-line "category action [args]" instead of
// nested menus. Typical usage:
//
//   > help
//   > pid show
//   > pid kp 15.0
//   > pid slow kp 18.0
//   > sound volume 22
//   > sound test 21
//   > imu mode fusion
//   > feature idle_anim off
//   > test servo head1 120
//   > save
//
// Non-blocking: cliUpdate() reads Serial chars into a buffer and only
// dispatches when a newline arrives. Every command returns quickly so the
// main loop keeps real-time guarantees (balance PID, IMU update).
//
// Apache 2.0 layering note: this file is new for the AIO32 branch and
// contains no BBDroids-derived code; it is our own straightforward CLI.

#include "cli.h"

// ---------------------------------------------------------------------------
// Line buffer + prompt state
// ---------------------------------------------------------------------------
static const size_t CLI_BUF_SIZE = 160;
static char     cliBuf[CLI_BUF_SIZE];
static size_t   cliBufLen = 0;
static bool     cliPromptPending = true;

// ---------------------------------------------------------------------------
// Debug live-stream state — set by `debug stream <kind> [ms]`, driven from
// cliUpdate(). STREAM_OFF = one-shot `debug rc` behaviour; anything else
// reprints the chosen payload every `debugStreamInterval` ms until the
// user types `debug stream off` (or any command, which stops the stream
// on the next keystroke arrival).
// ---------------------------------------------------------------------------
enum DebugStreamMode {
    STREAM_OFF     = 0,
    STREAM_RC      = 1,
    STREAM_IMU     = 2,
    STREAM_BUTTONS = 3,
    STREAM_BATTERY = 4
};
static uint8_t  debugStreamMode       = STREAM_OFF;
static uint32_t debugStreamIntervalMs = 200;
static uint32_t debugStreamLastPrint  = 0;

// ---------------------------------------------------------------------------
// Forward declarations (inside this .ino — visible from cliUpdate downward)
// ---------------------------------------------------------------------------
static void cliPrintPrompt();
static void cliHandleLine(char* line);
static int  cliTokenize(char* line, char* tokens[], int maxTokens);
static bool cliStreq(const char* a, const char* b);
static bool cliParseFloat(const char* s, float& out);
static bool cliParseInt  (const char* s, long& out);
static bool cliParseBool (const char* s, bool& out);

static void cliCmdHelp(int argc, char* argv[]);
static void cliCmdStatus();
static void cliCmdVersion();
static void cliCmdRestart();
static void cliCmdSave();
static void cliCmdLoad();
static void cliCmdReset(int argc, char* argv[]);

static void cliCmdPid(int argc, char* argv[]);
static void cliCmdSound(int argc, char* argv[]);
static void cliCmdDisplay(int argc, char* argv[]);
static void cliCmdImu(int argc, char* argv[]);
static void cliCmdFeature(int argc, char* argv[]);
static void cliCmdTest(int argc, char* argv[]);
static void cliCmdDebug(int argc, char* argv[]);
static void cliCmdBalance(int argc, char* argv[]);
static void cliCmdDrive(int argc, char* argv[]);
static void cliCmdRc(int argc, char* argv[]);

// Debug-dump payload printers (shared between one-shot `debug rc` and the
// live-streaming loop driven from cliUpdate()).
static void cliDumpRC();
static void cliDumpIMU();
static void cliDumpButtons();
static void cliDumpBattery();

// ===========================================================================
// PUBLIC API
// ===========================================================================

void cliBegin() {
    Serial.println();
    Serial.println(F("================================================"));
    Serial.println(F("D-O AIO32 Serial CLI ready"));
    Serial.println(F("Type 'help' for command categories."));
    Serial.println(F("================================================"));
    cliBufLen = 0;
    cliPromptPending = true;
}

void cliUpdate() {
    // Live-stream: emit the chosen debug payload every debugStreamIntervalMs
    // until the user types 'debug stream off'. A keystroke stops the stream
    // immediately (handled below in the read loop) so the user can break
    // out without having to type the off-command blind on top of a flood
    // of stream lines.
    if (debugStreamMode != STREAM_OFF) {
        uint32_t now = millis();
        if (now - debugStreamLastPrint >= debugStreamIntervalMs) {
            debugStreamLastPrint = now;
            switch (debugStreamMode) {
                case STREAM_RC:      cliDumpRC();      break;
                case STREAM_IMU:     cliDumpIMU();     break;
                case STREAM_BUTTONS: cliDumpButtons(); break;
                case STREAM_BATTERY: cliDumpBattery(); break;
                default: break;
            }
        }
    }

    // Print prompt once per command cycle, not every loop iteration.
    if (cliPromptPending) {
        cliPrintPrompt();
        cliPromptPending = false;
    }

    // Drain all available characters so big paste operations complete fast.
    while (Serial.available()) {
        int c = Serial.read();
        if (c < 0) break;

        // Any input keystroke while a live stream is active stops the
        // stream — makes it painless to break out of a scrolling monitor.
        if (debugStreamMode != STREAM_OFF) {
            debugStreamMode = STREAM_OFF;
            Serial.println();
            Serial.println(F("[stream off]"));
            cliPromptPending = true;
            // fall through and still process the character normally
        }

        // Handle line terminators — accept both \r\n and lone \n / \r.
        if (c == '\r' || c == '\n') {
            if (cliBufLen > 0) {
                cliBuf[cliBufLen] = '\0';
                Serial.println();         // echo newline
                cliHandleLine(cliBuf);
                cliBufLen = 0;
                cliPromptPending = true;
            } else if (c == '\n') {
                // empty line — re-prompt
                cliPromptPending = true;
            }
            continue;
        }

        // Backspace / DEL
        if (c == 0x08 || c == 0x7F) {
            if (cliBufLen > 0) {
                cliBufLen--;
                Serial.write("\b \b");  // visually erase
            }
            continue;
        }

        // Printable ASCII
        if (c >= 0x20 && c < 0x7F && cliBufLen + 1 < CLI_BUF_SIZE) {
            cliBuf[cliBufLen++] = (char)c;
            Serial.write((char)c);        // local echo
        }
    }
}

// ===========================================================================
// Line processing
// ===========================================================================

static void cliPrintPrompt() {
    Serial.print(F("D-O> "));
}

static bool cliStreq(const char* a, const char* b) {
    return strcasecmp(a, b) == 0;
}

static int cliTokenize(char* line, char* tokens[], int maxTokens) {
    int n = 0;
    char* p = line;
    while (*p && n < maxTokens) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') break;
        tokens[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) { *p = '\0'; p++; }
    }
    return n;
}

static bool cliParseFloat(const char* s, float& out) {
    char* endp = nullptr;
    double v = strtod(s, &endp);
    if (endp == s) return false;
    out = (float)v;
    return true;
}

static bool cliParseInt(const char* s, long& out) {
    char* endp = nullptr;
    long v = strtol(s, &endp, 0);
    if (endp == s) return false;
    out = v;
    return true;
}

static bool cliParseBool(const char* s, bool& out) {
    if (cliStreq(s, "on")   || cliStreq(s, "true")  || cliStreq(s, "1") ||
        cliStreq(s, "yes")  || cliStreq(s, "y")     || cliStreq(s, "enable")) {
        out = true;  return true;
    }
    if (cliStreq(s, "off")  || cliStreq(s, "false") || cliStreq(s, "0") ||
        cliStreq(s, "no")   || cliStreq(s, "n")     || cliStreq(s, "disable")) {
        out = false; return true;
    }
    return false;
}

static void cliHandleLine(char* line) {
    char* tokens[16];
    int argc = cliTokenize(line, tokens, 16);
    if (argc == 0) return;

    const char* cmd = tokens[0];

    if      (cliStreq(cmd, "help") || cliStreq(cmd, "?"))  cliCmdHelp(argc, tokens);
    else if (cliStreq(cmd, "status") || cliStreq(cmd, "st")) cliCmdStatus();
    else if (cliStreq(cmd, "version") || cliStreq(cmd, "ver")) cliCmdVersion();
    else if (cliStreq(cmd, "restart") || cliStreq(cmd, "reboot")) cliCmdRestart();
    else if (cliStreq(cmd, "save"))    cliCmdSave();
    else if (cliStreq(cmd, "load"))    cliCmdLoad();
    else if (cliStreq(cmd, "reset"))   cliCmdReset(argc, tokens);
    else if (cliStreq(cmd, "pid"))     cliCmdPid(argc, tokens);
    else if (cliStreq(cmd, "sound"))   cliCmdSound(argc, tokens);
    else if (cliStreq(cmd, "display")) cliCmdDisplay(argc, tokens);
    else if (cliStreq(cmd, "imu"))     cliCmdImu(argc, tokens);
    else if (cliStreq(cmd, "feature")) cliCmdFeature(argc, tokens);
    else if (cliStreq(cmd, "test"))    cliCmdTest(argc, tokens);
    else if (cliStreq(cmd, "debug"))   cliCmdDebug(argc, tokens);
    else if (cliStreq(cmd, "balance")) cliCmdBalance(argc, tokens);
    else if (cliStreq(cmd, "drive"))   cliCmdDrive(argc, tokens);
    else if (cliStreq(cmd, "rc"))      cliCmdRc(argc, tokens);
    else {
        Serial.print(F("Unknown command: "));
        Serial.println(cmd);
        Serial.println(F("Type 'help' for categories."));
    }
}

// ===========================================================================
// rc — RC protocol + diagnostics (runtime-switchable iBus/SBUS)
// ===========================================================================
static void cliCmdRc(int argc, char* argv[]) {
    if (argc < 2 || cliStreq(argv[1], "status")) {
        RCProtocol p = rcReceiver.getProtocol();
        Serial.print(F("RC protocol (active): "));
        Serial.println(RCReceiver::protocolName(p));
        Serial.print(F("RC protocol (stored): "));
        Serial.println(RCReceiver::protocolName(RCReceiver::loadStoredProtocol()));
        Serial.println(rcReceiver.getStatusString());
        return;
    }

    if (cliStreq(argv[1], "protocol")) {
        if (argc < 3) {
            Serial.print(F("Current protocol: "));
            Serial.println(RCReceiver::protocolName(rcReceiver.getProtocol()));
            Serial.println(F("Usage: rc protocol <ibus|sbus>"));
            return;
        }
        RCProtocol target;
        if (cliStreq(argv[2], "ibus")) {
            target = RC_PROTOCOL_IBUS;
        } else if (cliStreq(argv[2], "sbus")) {
            target = RC_PROTOCOL_SBUS;
        } else {
            Serial.println(F("Unknown protocol. Use 'ibus' or 'sbus'."));
            return;
        }

        // Runtime swap: end current UART, re-init with new protocol params.
        // ESP32 UART invert is applied inside rcReceiver.begin() — no
        // external hardware change needed on AIO32.
        rcReceiver.setProtocol(target);
        RCReceiver::storeProtocol(target);

        Serial.print(F("RC protocol switched to: "));
        Serial.println(RCReceiver::protocolName(target));
        Serial.println(F("Stored in NVS (persistent across reboot)."));
        Serial.println(F("Re-check with `rc status` after a few seconds."));
        return;
    }

    if (cliStreq(argv[1], "channels") || cliStreq(argv[1], "ch")) {
        rcReceiver.printChannels();
        return;
    }

    Serial.println(F("Usage:"));
    Serial.println(F("  rc                       — protocol + status"));
    Serial.println(F("  rc status                — same"));
    Serial.println(F("  rc protocol              — show active protocol"));
    Serial.println(F("  rc protocol ibus         — switch to FlySky iBus (no inverter)"));
    Serial.println(F("  rc protocol sbus         — switch to SBUS (ESP32 HW invert)"));
    Serial.println(F("  rc channels              — one-shot channel dump"));
}

// ===========================================================================
// help — top-level and per-category
// ===========================================================================

static void cliCmdHelp(int argc, char* argv[]) {
    if (argc < 2) {
        Serial.println(F("Categories:"));
        Serial.println(F("  help [category]   — detailed help"));
        Serial.println(F("  status            — full snapshot"));
        Serial.println(F("  version           — firmware identity"));
        Serial.println(F(""));
        Serial.println(F("  pid       balance PID gains, adaptive bands, autotune"));
        Serial.println(F("  sound     DFPlayer volume/mute/test-tracks"));
        Serial.println(F("  display   TFT rotation, mode, always-on"));
        Serial.println(F("  imu       mode selection, calibration, Madgwick toggle"));
        Serial.println(F("  drive     mixing mode (arcade/tank), deadband, expo, ramping, lean"));
        Serial.println(F("  rc        protocol switch (iBus/SBUS), channels, status"));
        Serial.println(F("  feature   runtime enable/disable flags"));
        Serial.println(F("  test      motor/servo/LED/sound manual test"));
        Serial.println(F("  debug     live data dumps (rc, imu, buttons)"));
        Serial.println(F("  balance   start/stop balance mode from CLI"));
        Serial.println(F(""));
        Serial.println(F("  save / load / reset factory / restart"));
        return;
    }
    const char* cat = argv[1];
    if (cliStreq(cat, "pid")) {
        Serial.println(F("pid commands:"));
        Serial.println(F("  pid show                        current gains + bands"));
        Serial.println(F("  pid kp <val>                    base Kp"));
        Serial.println(F("  pid ki <val>                    base Ki"));
        Serial.println(F("  pid kd <val>                    base Kd"));
        Serial.println(F("  pid slow kp <val>               slow-band Kp (|throttle|<0.2)"));
        Serial.println(F("  pid slow kd <val>               slow-band Kd"));
        Serial.println(F("  pid med  kp <val>               medium-band Kp (0.2..0.5)"));
        Serial.println(F("  pid med  kd <val>               medium-band Kd"));
        Serial.println(F("  pid fast kp <val>               fast-band Kp (|throttle|>0.8)"));
        Serial.println(F("  pid fast kd <val>               fast-band Kd"));
        Serial.println(F("  pid target <deg>                balance target angle"));
        Serial.println(F("  pid autotune                    start PID auto-tune"));
    } else if (cliStreq(cat, "sound")) {
        Serial.println(F("sound commands:"));
        Serial.println(F("  sound show                      current volume / mute / track counts"));
        Serial.println(F("  sound volume <0..30>            DFPlayer volume"));
        Serial.println(F("  sound mute / unmute             toggle mute flag"));
        Serial.println(F("  sound test <track#>             play specific track"));
        Serial.println(F("  sound stop                      stop current playback"));
    } else if (cliStreq(cat, "display")) {
        Serial.println(F("display commands:"));
        Serial.println(F("  display show                    current rotation + mode"));
        Serial.println(F("  display rotate <0..3>           set TFT rotation (persisted)"));
        Serial.println(F("  display rotate cycle            next orientation (like B5 short)"));
        Serial.println(F("  display mode telemetry|diag|off switch display page"));
        Serial.println(F("  display alwayson on|off         keep backlight on in running mode"));
    } else if (cliStreq(cat, "imu")) {
        Serial.println(F("imu commands:"));
        Serial.println(F("  imu show                        mode + live pitch/roll/yaw"));
        Serial.println(F("  imu mode qmi|lsm|fusion|auto    switch active IMU"));
        Serial.println(F("  imu cal                         run calibration (~5s, keep still)"));
        Serial.println(F("  imu bias show                   print calibration offsets"));
        Serial.println(F("  imu bias reset                  clear offsets"));
        Serial.println(F("  imu beta <val>                  Madgwick beta (0.03..0.5)"));
        Serial.println(F("  imu beta show                   current beta"));
    } else if (cliStreq(cat, "feature")) {
        Serial.println(F("feature commands:"));
        Serial.println(F("  feature show                    all flags"));
        Serial.println(F("  feature state_rx on|off         tilt warn + recovery sounds"));
        Serial.println(F("  feature idle_sound on|off       idle sound triggers"));
        Serial.println(F("  feature idle_anim on|off        idle servo animations"));
        Serial.println(F("  feature adaptive_pid on|off     3-band adaptive gains"));
        Serial.println(F("  feature sound on|off            master sound enable"));
        Serial.println(F("  feature madgwick on|off         runtime-toggleable (needs reboot)"));
    } else if (cliStreq(cat, "test")) {
        Serial.println(F("test commands:"));
        Serial.println(F("  test motor1 <-255..255>         spin motor 1"));
        Serial.println(F("  test motor2 <-255..255>         spin motor 2"));
        Serial.println(F("  test motors stop                stop both motors"));
        Serial.println(F("  test servo mainbar|head1..3 <0..180>"));
        Serial.println(F("  test servo center               all servos to 90"));
        Serial.println(F("  test led <r> <g> <b>            set NeoPixel (0..255 each)"));
        Serial.println(F("  test sound <track>              alias for 'sound test'"));
    } else if (cliStreq(cat, "debug")) {
        Serial.println(F("debug commands:"));
        Serial.println(F("  debug rc                        one-shot RC channel dump"));
        Serial.println(F("  debug imu                       one-shot raw IMU data dump"));
        Serial.println(F("  debug buttons                   one-shot button ADC ladder dump"));
        Serial.println(F("  debug batt                      battery voltage (if enabled)"));
        Serial.println(F("  debug stream <rc|imu|buttons|batt> [ms]  live-stream (default 200ms)"));
        Serial.println(F("  debug stream off                stop live-stream (any keystroke also stops)"));
    } else if (cliStreq(cat, "drive")) {
        Serial.println(F("drive commands:"));
        Serial.println(F("  drive show                      all driving-dynamic settings"));
        Serial.println(F("  drive mode arcade|tank          mixing mode"));
        Serial.println(F("  drive mode cycle                toggle between arcade/tank"));
        Serial.println(F("  drive shaping on|off            RC deadband + expo enable"));
        Serial.println(F("  drive deadband <0..0.5>         stick deadzone (normalised)"));
        Serial.println(F("  drive expo <0..1>               expo curve amount"));
        Serial.println(F("  drive ramping on|off            motor-output low-pass enable"));
        Serial.println(F("  drive ramp <0..1>               ramp rate (0.15 default, lower = smoother)"));
        Serial.println(F("  drive lean on|off               dynamic forward-lean enable"));
        Serial.println(F("  drive leanmax <0..15>           max lean angle (deg) at full throttle"));
    } else if (cliStreq(cat, "balance")) {
        Serial.println(F("balance commands:"));
        Serial.println(F("  balance start                   enter STATE_RUNNING (balance PID)"));
        Serial.println(F("  balance stop                    back to STATE_READY"));
        Serial.println(F("  balance estop                   emergency stop (motor cutoff)"));
        Serial.println(F("  balance baseline                auto-null target angle (B1-long)"));
    } else {
        Serial.print(F("No help for: "));
        Serial.println(cat);
    }
}

// ===========================================================================
// status — one-screen snapshot
// ===========================================================================

static void cliCmdStatus() {
    Serial.println(F("-- D-O AIO32 Status --"));
    // Balance PID
    Serial.printf("Balance : state=%d target=%.2f deg  angle=%.2f deg\n",
                  (int)currentState, balance.targetAngle, balance.currentAngle);
    Serial.printf("Base PID: Kp=%.2f Ki=%.2f Kd=%.2f  I-bound=%.1f  output=%.1f\n",
                  balance.kp, balance.ki, balance.kd,
                  (float)PID_I_BOUND, balance.pidOutput);
    Serial.printf("Adaptive: slow(%.2f/%.2f) med(%.2f/%.2f) fast(%.2f/%.2f)  thresh %.2f/%.2f/%.2f\n",
                  balance.kp_slow, balance.kd_slow,
                  balance.kp_medium, balance.kd_medium,
                  balance.kp_fast, balance.kd_fast,
                  (float)ADAPTIVE_THRESH_SLOW, (float)ADAPTIVE_THRESH_MED, (float)ADAPTIVE_THRESH_FAST);

    // IMU
    IMUStatus imus = imuHandler.getStatus();
    Serial.printf("IMU     : mode=%d  QMI=%s LSM=%s  rate=%.1fHz\n",
                  (int)imus.active_mode,
                  imus.qmi_initialized ? "OK" : "--",
                  imus.lsm_initialized ? "OK" : "--",
                  imus.update_rate);
    Serial.printf("Orient  : P=%.2f R=%.2f Y=%.2f  (deg)\n",
                  imuHandler.getPitch(), imuHandler.getRoll(), imuHandler.getYaw());

    // RC
    Serial.printf("RC      : connected=%s  steer=%.2f thr=%.2f headT=%.2f headP=%.2f\n",
                  rcControl.isConnected ? "yes" : "NO",
                  rcControl.steering, rcControl.throttle,
                  rcControl.headTilt, rcControl.headPan);

    // Sound
    DFPlayerStatus snds = soundController.getStatus();
    Serial.printf("Sound   : enabled=%s init=%s card=%s playing=%s vol=%u\n",
                  soundEnabled ? "yes" : "no",
                  snds.initialized ? "yes" : "no",
                  snds.cardOnline  ? "yes" : "no",
                  snds.playing     ? "yes" : "no",
                  snds.volume);

    // Display
    Serial.printf("Display : mode=%u  alwaysOn=%s\n",
                  (unsigned)displayMode, displayAlwaysOn ? "yes" : "no");

    // Drive dynamics
    Serial.printf("Drive   : %s  shape=%s(db=%.2f,expo=%.2f)  ramp=%s(%.2f)  lean=%s(max=%.1f)\n",
                  driveMode == DRIVE_MODE_TANK ? "Tank" : "Arcade",
                  rcShapingEnabled    ? "on" : "off", rcDeadband, rcExpo,
                  motorRampingEnabled ? "on" : "off", motorRampRate,
                  dynamicAngleEnabled ? "on" : "off", maxLeanAngle);

    // Feature flags
    Serial.printf("Features: state_rx=%s idle_sound=%s idle_anim=%s adaptive_pid=%s\n",
                  stateReactionsEnabled ? "on" : "off",
                  idleActionsEnabled    ? "on" : "off",
                  idleAnimationsEnabled ? "on" : "off",
                  ADAPTIVE_PID_ENABLED  ? "on(compile)" : "off(compile)");

    // System
    Serial.printf("System  : uptime=%lus free=%lu KB\n",
                  (unsigned long)(millis() / 1000),
                  (unsigned long)(ESP.getFreeHeap() / 1024));
}

// ===========================================================================
// version / restart / save / load / reset
// ===========================================================================

static void cliCmdVersion() {
    Serial.println(F("D-O AIO32 v2.1.0 firmware"));
    Serial.println(F("Build: " __DATE__ " " __TIME__));
}

static void cliCmdRestart() {
    Serial.println(F("Restarting..."));
    delay(200);
    ESP.restart();
}

static void cliCmdSave() {
    savePreferences();
    Serial.println(F("Saved."));
}

static void cliCmdLoad() {
    loadPreferences();
    initializeBalancePid();
    Serial.println(F("Loaded."));
}

static void cliCmdReset(int argc, char* argv[]) {
    if (argc < 2 || !cliStreq(argv[1], "factory")) {
        Serial.println(F("Usage: reset factory   (this clears ALL NVS and reboots)"));
        return;
    }
    Serial.println(F("Factory reset in 2 seconds..."));
    delay(2000);
    performFactoryReset();   // clears NVS, restarts
}

// ===========================================================================
// pid — gain tuning
// ===========================================================================

static void cliCmdPid(int argc, char* argv[]) {
    if (argc < 2 || cliStreq(argv[1], "show")) {
        Serial.printf("Base : Kp=%.2f Ki=%.2f Kd=%.2f  target=%.2f deg\n",
                      balance.kp, balance.ki, balance.kd, balance.targetAngle);
        Serial.printf("Slow : Kp=%.2f Kd=%.2f  (|thr|<=%.2f)\n",
                      balance.kp_slow, balance.kd_slow, (float)ADAPTIVE_THRESH_SLOW);
        Serial.printf("Med  : Kp=%.2f Kd=%.2f  (%.2f..%.2f)\n",
                      balance.kp_medium, balance.kd_medium,
                      (float)ADAPTIVE_THRESH_SLOW, (float)ADAPTIVE_THRESH_MED);
        Serial.printf("Fast : Kp=%.2f Kd=%.2f  (|thr|>=%.2f)\n",
                      balance.kp_fast, balance.kd_fast, (float)ADAPTIVE_THRESH_FAST);
        return;
    }

    const char* sub = argv[1];

    if (cliStreq(sub, "kp") && argc >= 3) {
        float v; if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        balance.kp = v; initializeBalancePid();
        Serial.printf("Base Kp = %.2f\n", balance.kp); return;
    }
    if (cliStreq(sub, "ki") && argc >= 3) {
        float v; if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        balance.ki = v; initializeBalancePid();
        Serial.printf("Base Ki = %.2f\n", balance.ki); return;
    }
    if (cliStreq(sub, "kd") && argc >= 3) {
        float v; if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        balance.kd = v; initializeBalancePid();
        Serial.printf("Base Kd = %.2f\n", balance.kd); return;
    }
    if (cliStreq(sub, "target") && argc >= 3) {
        float v; if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        balance.targetAngle = v;
        Serial.printf("Target angle = %.2f deg\n", balance.targetAngle); return;
    }
    if (cliStreq(sub, "autotune")) {
        Serial.println(F("Starting Auto-Tune (droid must be balancing on hardware)"));
        startAutoTune();
        return;
    }

    // Adaptive bands: "pid slow kp 15.0" etc.
    if ((cliStreq(sub, "slow") || cliStreq(sub, "med") || cliStreq(sub, "fast")) && argc >= 4) {
        const char* band  = sub;
        const char* which = argv[2];
        float v;
        if (!cliParseFloat(argv[3], v)) { Serial.println(F("bad value")); return; }

        float* target = nullptr;
        if      (cliStreq(band, "slow") && cliStreq(which, "kp")) target = &balance.kp_slow;
        else if (cliStreq(band, "slow") && cliStreq(which, "kd")) target = &balance.kd_slow;
        else if (cliStreq(band, "med")  && cliStreq(which, "kp")) target = &balance.kp_medium;
        else if (cliStreq(band, "med")  && cliStreq(which, "kd")) target = &balance.kd_medium;
        else if (cliStreq(band, "fast") && cliStreq(which, "kp")) target = &balance.kp_fast;
        else if (cliStreq(band, "fast") && cliStreq(which, "kd")) target = &balance.kd_fast;

        if (!target) { Serial.println(F("bad band/param")); return; }
        *target = v;
        Serial.printf("%s %s = %.2f\n", band, which, v);
        return;
    }

    Serial.println(F("Usage: help pid"));
}

// ===========================================================================
// sound
// ===========================================================================

static void cliCmdSound(int argc, char* argv[]) {
    if (argc < 2 || cliStreq(argv[1], "show")) {
        DFPlayerStatus s = soundController.getStatus();
        Serial.printf("Sound: enabled=%s init=%s card=%s playing=%s vol=%u muted=%s\n",
                      soundEnabled       ? "yes" : "no",
                      s.initialized      ? "yes" : "no",
                      s.cardOnline       ? "yes" : "no",
                      s.playing          ? "yes" : "no",
                      s.volume,
                      soundController.isMuted() ? "yes" : "no");
        return;
    }
    const char* sub = argv[1];
    if (cliStreq(sub, "volume") && argc >= 3) {
        long v; if (!cliParseInt(argv[2], v)) { Serial.println(F("bad value")); return; }
        if (v < 0)  v = 0;
        if (v > 30) v = 30;
        soundController.setVolume((uint8_t)v);
        Serial.printf("Volume = %ld\n", v);
        return;
    }
    if (cliStreq(sub, "mute"))   { soundController.setMute(true);  Serial.println(F("Muted.")); return; }
    if (cliStreq(sub, "unmute")) { soundController.setMute(false); Serial.println(F("Unmuted.")); return; }
    if (cliStreq(sub, "stop"))   { soundController.stop();         Serial.println(F("Stopped.")); return; }
    if (cliStreq(sub, "test") && argc >= 3) {
        long track; if (!cliParseInt(argv[2], track)) { Serial.println(F("bad track")); return; }
        if (track < 1 || track > 255) { Serial.println(F("track out of range")); return; }
        soundController.playSound((uint8_t)track);
        Serial.printf("Playing track %ld\n", track);
        return;
    }
    Serial.println(F("Usage: help sound"));
}

// ===========================================================================
// display
// ===========================================================================

static void cliCmdDisplay(int argc, char* argv[]) {
    if (argc < 2 || cliStreq(argv[1], "show")) {
        Serial.printf("Display: mode=%u  alwaysOn=%s\n",
                      (unsigned)displayMode, displayAlwaysOn ? "yes" : "no");
        return;
    }
    const char* sub = argv[1];
    if (cliStreq(sub, "rotate") && argc >= 3) {
        if (cliStreq(argv[2], "cycle")) {
            cycleDriveMode();  // our B5-short handler — cycles + persists
            return;
        }
        long v; if (!cliParseInt(argv[2], v)) { Serial.println(F("bad value")); return; }
        uint8_t rot = (uint8_t)(v & 0x03);
        displayHandler.setRotation(rot);
        systemPrefs.begin("d-o-droid", false);
        systemPrefs.putUChar("tftRot", rot);
        systemPrefs.end();
        Serial.printf("Rotation = %u (saved)\n", rot);
        return;
    }
    if (cliStreq(sub, "mode") && argc >= 3) {
        const char* m = argv[2];
        if      (cliStreq(m, "telemetry") || cliStreq(m, "tel"))  { displayMode = 0; displayHandler.setMode(DISPLAY_MODE_TELEMETRY); }
        else if (cliStreq(m, "diagnostics") || cliStreq(m, "diag") || cliStreq(m, "di")) { displayMode = 1; displayHandler.setMode(DISPLAY_MODE_DIAGNOSTICS); }
        else if (cliStreq(m, "off"))      { displayMode = 2; displayHandler.setBacklight(false); }
        else { Serial.println(F("bad mode (telemetry|diag|off)")); return; }
        Serial.printf("Display mode = %s\n", m);
        return;
    }
    if (cliStreq(sub, "alwayson") && argc >= 3) {
        bool v; if (!cliParseBool(argv[2], v)) { Serial.println(F("bad value")); return; }
        displayAlwaysOn = v;
        Serial.printf("AlwaysOn = %s\n", v ? "on" : "off");
        return;
    }
    Serial.println(F("Usage: help display"));
}

// ===========================================================================
// imu
// ===========================================================================

static void cliCmdImu(int argc, char* argv[]) {
    if (argc < 2 || cliStreq(argv[1], "show")) {
        IMUStatus s = imuHandler.getStatus();
        Serial.printf("IMU: mode=%d qmi=%s lsm=%s rate=%.1fHz\n",
                      (int)s.active_mode,
                      s.qmi_initialized ? "OK" : "FAIL",
                      s.lsm_initialized ? "OK" : "FAIL",
                      s.update_rate);
        Serial.printf("     pitch=%.2f roll=%.2f yaw=%.2f (deg)\n",
                      imuHandler.getPitch(), imuHandler.getRoll(), imuHandler.getYaw());
        return;
    }
    const char* sub = argv[1];
    if (cliStreq(sub, "mode") && argc >= 3) {
        const char* m = argv[2];
        IMUMode target = IMU_MODE_AUTO;
        bool ok = true;
        if      (cliStreq(m, "qmi"))    target = IMU_MODE_QMI_ONLY;
        else if (cliStreq(m, "lsm"))    target = IMU_MODE_LSM_ONLY;
        else if (cliStreq(m, "fusion")) target = IMU_MODE_FUSION;
        else if (cliStreq(m, "auto"))   target = IMU_MODE_AUTO;
        else ok = false;
        if (!ok) { Serial.println(F("bad mode (qmi|lsm|fusion|auto)")); return; }

        // AUTO is only meaningful during begin(); run the live cycle helper
        // to pick the best concrete available mode.
        if (target == IMU_MODE_AUTO) {
            cycleIMUMode();
            return;
        }
        if (imuHandler.setMode(target)) {
            Serial.printf("IMU mode = %s\n", m);
        } else {
            Serial.println(F("setMode refused (sensor not initialised)"));
        }
        return;
    }
    if (cliStreq(sub, "cal")) {
        Serial.println(F("Calibrating IMU — keep the droid still for ~5 seconds..."));
        imuHandler.calibrate(1000);
        imuHandler.saveCalibration();
        Serial.println(F("Calibration done + saved."));
        return;
    }
    if (cliStreq(sub, "bias")) {
        if (argc >= 3 && cliStreq(argv[2], "reset")) {
            imuHandler.resetCalibration();
            imuHandler.saveCalibration();
            Serial.println(F("Bias reset + saved."));
            return;
        }
        // default subcmd = show
        imuHandler.printBias(Serial);
        return;
    }
#if IMU_USE_MADGWICK
    if (cliStreq(sub, "beta")) {
        if (argc < 3 || cliStreq(argv[2], "show")) {
            Serial.printf("Madgwick beta = %.3f  (config default %.3f)\n",
                          imuHandler.getMadgwickBeta(), (float)IMU_MADGWICK_BETA);
            return;
        }
        float v;
        if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        if (v < 0.0f)  v = 0.0f;
        if (v > 2.0f)  v = 2.0f;  // hard ceiling, values >0.5 are already extreme
        imuHandler.setMadgwickBeta(v);
        // Persist so it survives reboot — CLI users can revert by calling
        // 'imu beta 0.1' or flashing with a different IMU_MADGWICK_BETA.
        systemPrefs.begin("d-o-droid", false);
        systemPrefs.putFloat("imuBeta", v);
        systemPrefs.end();
        Serial.printf("Madgwick beta = %.3f (saved)\n", v);
        return;
    }
#endif
    Serial.println(F("Usage: help imu"));
}

// ===========================================================================
// feature — runtime toggles
// ===========================================================================

static void cliCmdFeature(int argc, char* argv[]) {
    if (argc < 2 || cliStreq(argv[1], "show")) {
        Serial.printf("state_rx     = %s\n", stateReactionsEnabled ? "on" : "off");
        Serial.printf("idle_sound   = %s\n", idleActionsEnabled    ? "on" : "off");
        Serial.printf("idle_anim    = %s\n", idleAnimationsEnabled ? "on" : "off");
        Serial.printf("sound        = %s (master)\n", soundEnabled  ? "on" : "off");
        Serial.printf("adaptive_pid = %s (compile-time)\n", ADAPTIVE_PID_ENABLED ? "on" : "off");
        Serial.printf("madgwick     = %s (compile-time)\n", IMU_USE_MADGWICK ? "on" : "off");
        return;
    }
    if (argc < 3) { Serial.println(F("Usage: feature <name> on|off")); return; }
    const char* name = argv[1];
    bool v;
    if (!cliParseBool(argv[2], v)) { Serial.println(F("bad value (on|off)")); return; }

    if      (cliStreq(name, "state_rx"))   { stateReactionsEnabled = v; }
    else if (cliStreq(name, "idle_sound")) { idleActionsEnabled    = v; }
    else if (cliStreq(name, "idle_anim"))  { idleAnimationsEnabled = v; }
    else if (cliStreq(name, "sound"))      { soundEnabled          = v; soundController.setMute(!v); }
    else {
        Serial.println(F("Unknown feature. Try 'feature show'."));
        return;
    }
    Serial.printf("%s = %s (not yet saved — use 'save' to persist)\n", name, v ? "on" : "off");
}

// ===========================================================================
// test — manual hardware exercising
// ===========================================================================

static void cliCmdTest(int argc, char* argv[]) {
    if (argc < 2) { Serial.println(F("Usage: help test")); return; }
    const char* sub = argv[1];

    if ((cliStreq(sub, "motor1") || cliStreq(sub, "motor2")) && argc >= 3) {
        if (cliStreq(argv[2], "stop")) {
            stopMotors();
            Serial.println(F("Motors stopped."));
            return;
        }
        long v; if (!cliParseInt(argv[2], v)) { Serial.println(F("bad value")); return; }
        if (v < -255) v = -255;
        if (v >  255) v =  255;
        int pwmPin = cliStreq(sub, "motor1") ? MOTOR1_PWM_PIN : MOTOR2_PWM_PIN;
        int dirPin = cliStreq(sub, "motor1") ? MOTOR1_DIR_PIN : MOTOR2_DIR_PIN;
        digitalWrite(dirPin, v >= 0 ? 1 : 0);
        ledcWrite(pwmPin, abs((int)v));
        Serial.printf("%s = %ld\n", sub, v);
        return;
    }
    if (cliStreq(sub, "motors") && argc >= 3 && cliStreq(argv[2], "stop")) {
        stopMotors();
        Serial.println(F("Motors stopped."));
        return;
    }

    if (cliStreq(sub, "servo") && argc >= 3) {
        if (cliStreq(argv[2], "center")) {
            servoWrite(SERVO_MAINBAR_PIN, 90);
            servoWrite(SERVO_HEAD1_PIN,   90);
            servoWrite(SERVO_HEAD2_PIN,   90);
            servoWrite(SERVO_HEAD3_PIN,   90);
            Serial.println(F("All servos -> 90 deg"));
            return;
        }
        if (argc < 4) { Serial.println(F("Usage: test servo <mainbar|head1|head2|head3|center> [angle]")); return; }
        long angle; if (!cliParseInt(argv[3], angle)) { Serial.println(F("bad angle")); return; }
        int pin = -1;
        if      (cliStreq(argv[2], "mainbar")) pin = SERVO_MAINBAR_PIN;
        else if (cliStreq(argv[2], "head1"))   pin = SERVO_HEAD1_PIN;
        else if (cliStreq(argv[2], "head2"))   pin = SERVO_HEAD2_PIN;
        else if (cliStreq(argv[2], "head3"))   pin = SERVO_HEAD3_PIN;
        if (pin < 0) { Serial.println(F("unknown servo")); return; }
        if (angle < 0)   angle = 0;
        if (angle > 180) angle = 180;
        servoWrite((uint8_t)pin, (int)angle);
        Serial.printf("servo %s -> %ld deg\n", argv[2], angle);
        return;
    }

    if (cliStreq(sub, "led") && argc >= 5) {
        long r, g, b;
        if (!cliParseInt(argv[2], r) || !cliParseInt(argv[3], g) || !cliParseInt(argv[4], b)) {
            Serial.println(F("bad rgb"));
            return;
        }
        utilities.setLED(LEDColor((uint8_t)r, (uint8_t)g, (uint8_t)b));
        Serial.printf("LED = %ld/%ld/%ld\n", r, g, b);
        return;
    }

    if (cliStreq(sub, "sound") && argc >= 3) {
        // Alias for `sound test <track>`
        long track; if (!cliParseInt(argv[2], track)) { Serial.println(F("bad track")); return; }
        soundController.playSound((uint8_t)track);
        Serial.printf("Playing track %ld\n", track);
        return;
    }

    Serial.println(F("Usage: help test"));
}

// ===========================================================================
// debug — one-shot live dumps
// ===========================================================================

// Individual payload-printers — reused by one-shot commands + live stream.
static void cliDumpRC() {
    Serial.printf("RC connected=%s  steer=%+.3f throttle=%+.3f headTilt=%+.3f headPan=%+.3f  mainbar=%+.3f head3=%+.3f  mute=%s\n",
                  rcControl.isConnected ? "yes" : "NO",
                  rcControl.steering, rcControl.throttle,
                  rcControl.headTilt, rcControl.headPan,
                  rcReceiver.getMainbar(), rcReceiver.getHead3(),
                  rcControl.mute ? "on" : "off");
}

static void cliDumpIMU() {
    RawIMUData qmi = imuHandler.getQMIData();
    RawIMUData lsm = imuHandler.getLSMData();
    Serial.printf("QMI acc=%+.2f,%+.2f,%+.2f  gyr=%+.2f,%+.2f,%+.2f  T=%.1fC | "
                  "LSM acc=%+.2f,%+.2f,%+.2f  gyr=%+.2f,%+.2f,%+.2f  T=%.1fC | "
                  "orient P=%+.2f R=%+.2f Y=%+.2f\n",
                  qmi.ax, qmi.ay, qmi.az, qmi.gx, qmi.gy, qmi.gz, qmi.temperature,
                  lsm.ax, lsm.ay, lsm.az, lsm.gx, lsm.gy, lsm.gz, lsm.temperature,
                  imuHandler.getPitch(), imuHandler.getRoll(), imuHandler.getYaw());
}

static void cliDumpButtons() {
    uint16_t l0 = buttonHandler.getLadderRaw(0);
    uint16_t l1 = buttonHandler.getLadderRaw(1);
    Serial.printf("ADC1(pin%d)=%4u ADC2(pin%d)=%4u  pressed:",
                  (int)BUTTON_LADDER1_PIN, l0, (int)BUTTON_LADDER2_PIN, l1);
    for (uint8_t i = 1; i <= 6; ++i) {
        if (buttonHandler.isButtonPressed(i)) Serial.printf(" B%u", i);
    }
    Serial.println();
}

static void cliDumpBattery() {
    Serial.printf("Battery: %.2f V  warn=%s critical=%s (monitor %s)\n",
                  batteryVoltage,
                  batteryWarnActive     ? "YES" : "no",
                  batteryCriticalActive ? "YES (latched)" : "no",
                  BATTERY_MONITOR_ENABLED ? "enabled" : "DISABLED in config.h");
}

static void cliCmdDebug(int argc, char* argv[]) {
    if (argc < 2) { Serial.println(F("Usage: help debug")); return; }
    const char* sub = argv[1];

    if (cliStreq(sub, "rc"))      { cliDumpRC();      return; }
    if (cliStreq(sub, "imu"))     { cliDumpIMU();     return; }
    if (cliStreq(sub, "buttons")) { cliDumpButtons(); return; }
    if (cliStreq(sub, "batt"))    { cliDumpBattery(); return; }

    // Live-stream mode:
    //   debug stream <rc|imu|buttons|batt> [ms]
    //   debug stream off
    if (cliStreq(sub, "stream")) {
        if (argc < 3) {
            Serial.println(F("Usage: debug stream <rc|imu|buttons|batt|off> [interval_ms]"));
            return;
        }
        const char* kind = argv[2];
        if (cliStreq(kind, "off") || cliStreq(kind, "stop")) {
            debugStreamMode = STREAM_OFF;
            Serial.println(F("Stream off"));
            return;
        }
        // Interval (optional, default 200 ms, min 50 ms)
        uint32_t interval = 200;
        if (argc >= 4) {
            long v;
            if (cliParseInt(argv[3], v) && v > 0) {
                if (v < 50) v = 50;
                interval = (uint32_t)v;
            }
        }
        uint8_t m = STREAM_OFF;
        // Per-kind default interval: buttons fires fast so short presses
        // (< 100 ms debounce window) actually get captured. Battery can
        // stay slow.
        uint32_t defaultIntervalMs = 200;
        if      (cliStreq(kind, "rc"))      { m = STREAM_RC;      defaultIntervalMs = 200;  }
        else if (cliStreq(kind, "imu"))     { m = STREAM_IMU;     defaultIntervalMs = 200;  }
        else if (cliStreq(kind, "buttons")) { m = STREAM_BUTTONS; defaultIntervalMs = 50;   }
        else if (cliStreq(kind, "batt"))    { m = STREAM_BATTERY; defaultIntervalMs = 1000; }
        else {
            Serial.println(F("Unknown stream kind (rc|imu|buttons|batt|off)"));
            return;
        }

        // If the user did NOT pass an explicit interval, honor the
        // per-kind default. The generic `200` at the top of this branch
        // was the fallback we want to override here.
        if (argc < 4) interval = defaultIntervalMs;

        debugStreamMode       = m;
        debugStreamIntervalMs = interval;
        debugStreamLastPrint  = 0;   // force immediate first print

        // Flush any pending serial chars that arrived with the command
        // line itself (trailing \r, \n on Windows line-endings, etc.) —
        // otherwise the "any keystroke stops stream" guard in cliUpdate()
        // would kill the stream immediately on the very next iteration
        // because these chars are still in the UART RX FIFO.
        while (Serial.available()) Serial.read();

        Serial.printf("Streaming %s every %lu ms. Press any key (or 'debug stream off') to stop.\n",
                      kind, (unsigned long)interval);
        return;
    }

    Serial.println(F("Usage: help debug"));
}

// ===========================================================================
// drive — driving-dynamics tuning (mixing mode, shaping, ramping, lean)
// ===========================================================================

static void cliCmdDrive(int argc, char* argv[]) {
    if (argc < 2 || cliStreq(argv[1], "show")) {
        Serial.printf("Mixing  : %s\n",
                      driveMode == DRIVE_MODE_TANK ? "Tank" : "Arcade");
        Serial.printf("Shaping : %s   deadband=%.2f  expo=%.2f\n",
                      rcShapingEnabled ? "on" : "off",
                      rcDeadband, rcExpo);
        Serial.printf("Ramping : %s   rate=%.2f (0.15=Mega default, lower=smoother)\n",
                      motorRampingEnabled ? "on" : "off", motorRampRate);
        Serial.printf("Lean    : %s   max=%.1f deg @ full throttle\n",
                      dynamicAngleEnabled ? "on" : "off", maxLeanAngle);
        return;
    }

    const char* sub = argv[1];

    if (cliStreq(sub, "mode") && argc >= 3) {
        const char* m = argv[2];
        if (cliStreq(m, "arcade"))      { driveMode = DRIVE_MODE_ARCADE; }
        else if (cliStreq(m, "tank"))   { driveMode = DRIVE_MODE_TANK; }
        else if (cliStreq(m, "cycle"))  { driveMode = (driveMode == DRIVE_MODE_ARCADE) ? DRIVE_MODE_TANK : DRIVE_MODE_ARCADE; }
        else { Serial.println(F("bad mode (arcade|tank|cycle)")); return; }
        Serial.printf("Drive mode = %s\n",
                      driveMode == DRIVE_MODE_TANK ? "Tank" : "Arcade");
        return;
    }

    if (cliStreq(sub, "shaping") && argc >= 3) {
        bool v; if (!cliParseBool(argv[2], v)) { Serial.println(F("bad value")); return; }
        rcShapingEnabled = v;
        Serial.printf("Shaping = %s\n", v ? "on" : "off");
        return;
    }
    if (cliStreq(sub, "deadband") && argc >= 3) {
        float v; if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        if (v < 0.0f) v = 0.0f;
        if (v > 0.5f) v = 0.5f;   // anything over 50 % deadzone is nonsense
        rcDeadband = v;
        Serial.printf("Deadband = %.3f\n", rcDeadband);
        return;
    }
    if (cliStreq(sub, "expo") && argc >= 3) {
        float v; if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        rcExpo = v;
        Serial.printf("Expo = %.2f\n", rcExpo);
        return;
    }

    if (cliStreq(sub, "ramping") && argc >= 3) {
        bool v; if (!cliParseBool(argv[2], v)) { Serial.println(F("bad value")); return; }
        motorRampingEnabled = v;
        Serial.printf("Ramping = %s\n", v ? "on" : "off");
        return;
    }
    if (cliStreq(sub, "ramp") && argc >= 3) {
        float v; if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        if (v < 0.01f) v = 0.01f;
        if (v > 1.0f)  v = 1.0f;
        motorRampRate = v;
        Serial.printf("Ramp rate = %.3f\n", motorRampRate);
        return;
    }

    if (cliStreq(sub, "lean") && argc >= 3) {
        bool v; if (!cliParseBool(argv[2], v)) { Serial.println(F("bad value")); return; }
        dynamicAngleEnabled = v;
        Serial.printf("Dynamic lean = %s\n", v ? "on" : "off");
        return;
    }
    if (cliStreq(sub, "leanmax") && argc >= 3) {
        float v; if (!cliParseFloat(argv[2], v)) { Serial.println(F("bad value")); return; }
        if (v < 0.0f)  v = 0.0f;
        if (v > 15.0f) v = 15.0f;   // above 15° the droid just tips over
        maxLeanAngle = v;
        Serial.printf("Max lean = %.1f deg\n", maxLeanAngle);
        return;
    }

    Serial.println(F("Usage: help drive"));
}

// ===========================================================================
// balance — state control from CLI
// ===========================================================================

static void cliCmdBalance(int argc, char* argv[]) {
    if (argc < 2) { Serial.println(F("Usage: help balance")); return; }
    const char* sub = argv[1];
    if (cliStreq(sub, "start")) {
        if (currentState == STATE_EMERGENCY_STOP) {
            Serial.println(F("Cannot start from EMERGENCY_STOP. Use 'restart' first."));
            return;
        }
        currentState = STATE_RUNNING;
        Serial.println(F("Entering STATE_RUNNING."));
        return;
    }
    if (cliStreq(sub, "stop")) {
        currentState = STATE_READY;
        stopMotors();
        Serial.println(F("Back to STATE_READY."));
        return;
    }
    if (cliStreq(sub, "estop")) {
        emergencyStop();
        Serial.println(F("Emergency stop triggered."));
        return;
    }
    if (cliStreq(sub, "baseline")) {
        runBaselineCalibration();
        return;
    }
    Serial.println(F("Usage: help balance"));
}
