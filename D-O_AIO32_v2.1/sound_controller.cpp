/**
 * Sound Controller — Makuna DFMiniMp3 backend
 *
 * Replaces the previous DFRobot/DFRobotDFPlayerMini implementation. The
 * Makuna library is non-blocking (no handshake wait in begin()), reports
 * status via static callbacks on Mp3Notify, and is the recommended choice
 * for ESP32/ESP32-S3 with HardwareSerial.
 *
 * Tracks are expected as /mp3/NNNN.mp3 on the SD card (Makuna convention),
 * which matches the existing Mega-line convention so the SD card layout
 * does NOT need to change.
 */

#include "sound_controller.h"
#include "esp_task_wdt.h"

// Global instance
SoundController soundController;

// ===========================================================================
// Mp3Notify — static callbacks invoked by the DFMiniMp3 template on
// error, SD-online/removed, and play-finished events. We mirror the
// interesting bits into soundController.status so the rest of the sketch
// can query a plain status struct.
// ===========================================================================

void Mp3Notify::PrintlnSourceAction(DfMp3_PlaySources source, const char* action) {
    // Informational: log which source (SD/USB/Flash) triggered the action.
    Serial.print("[DFMp3] ");
    if (source & DfMp3_PlaySources_Sd)    Serial.print("SD ");
    if (source & DfMp3_PlaySources_Usb)   Serial.print("USB ");
    if (source & DfMp3_PlaySources_Flash) Serial.print("Flash ");
    Serial.println(action);
}

void Mp3Notify::OnError(DfMp3& /*mp3*/, uint16_t errorCode) {
    Serial.printf("[DFMp3] error %u\n", errorCode);
    soundController.status.lastError    = millis();
    soundController.status.errorMessage = String("DFMp3 error ") + errorCode;
    // Error 1 = "busy/no SD card" on most DFPlayer clones — mark card offline.
    if (errorCode == 1) {
        soundController.status.cardOnline = false;
    }
}

void Mp3Notify::OnPlayFinished(DfMp3& /*mp3*/, DfMp3_PlaySources /*source*/, uint16_t track) {
    Serial.printf("[DFMp3] play finished: track %u\n", track);
    soundController.status.playing = false;
}

void Mp3Notify::OnPlaySourceOnline(DfMp3& /*mp3*/, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "online");
    if (source & DfMp3_PlaySources_Sd) soundController.status.cardOnline = true;
}

void Mp3Notify::OnPlaySourceInserted(DfMp3& /*mp3*/, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "inserted");
    if (source & DfMp3_PlaySources_Sd) soundController.status.cardOnline = true;
}

void Mp3Notify::OnPlaySourceRemoved(DfMp3& /*mp3*/, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "removed");
    if (source & DfMp3_PlaySources_Sd) soundController.status.cardOnline = false;
}

// ===========================================================================
// SoundController
// ===========================================================================

// Constructor
SoundController::SoundController() :
    dfPlayer(nullptr),
    playerSerial(nullptr),
    muted(false),
    queueHead(0),
    queueTail(0),
    queueCount(0),
    lastSoundTime(0),
    lastStatusCheck(0) {
    memset(&status, 0, sizeof(DFPlayerStatus));
    memset(&channelStates, 0, sizeof(channelStates));
}

SoundController::~SoundController() {
    end();
}

// -------------------------------------------------------------------------
// begin() — strictly non-blocking wait for SD online.
//
// Rationale: previous implementation called dfPlayer->reset() and
// setPlaybackSource() synchronously during setup(). When the DFPlayer is
// absent, has no SD, or responds with a reset-storm, those calls trigger
// ESP_RST_INT_WDT (interrupt watchdog, reset reason 11) because the
// library blocks inside its command-ack loop while the main loop() is
// not yet running, so notifications never get processed.
//
// New approach:
//   - Allocate + begin() only (fast, non-blocking).
//   - DO NOT call reset() in setup. The DFPlayer powers up in a known
//     state; forcing reset is the #1 hang trigger on stubborn clones.
//   - Pump dfPlayer->loop() actively during a bounded settling window,
//     watching for Mp3Notify::OnPlaySourceOnline to set status.cardOnline.
//   - If the window expires without SD online, keep status.initialized
//     = false and return false. Sketch continues without sound, no crash.
//   - setVolume / setEq only after SD-online confirmation.
// -------------------------------------------------------------------------
bool SoundController::begin() {
    Serial.println("  [CP8a] reset status fields"); Serial.flush();
    status.initialized   = false;
    status.cardOnline    = false;
    status.playing       = false;
    status.totalTracks   = 0;
    status.errorMessage  = "";
    status.volume        = DEFAULT_VOLUME;

    Serial.printf("  [CP8b] Serial2.begin(9600) RX=%d TX=%d\n",
                  DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
    Serial.flush();
    playerSerial = &Serial2;
    playerSerial->begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
    Serial.println("  [CP8b] OK"); Serial.flush();
    delay(50);
    esp_task_wdt_reset();

    Serial.println("  [CP8c] allocate DfMp3 + begin()"); Serial.flush();
    if (dfPlayer) { delete dfPlayer; dfPlayer = nullptr; }
    dfPlayer = new DfMp3(*playerSerial);
    if (!dfPlayer) {
        Serial.println("  [CP8c] alloc failed"); Serial.flush();
        status.errorMessage = "DfMp3 alloc failed";
        return false;
    }
    dfPlayer->begin();
    Serial.println("  [CP8c] OK"); Serial.flush();

    // Send an async reset command. Unlike the DFRobot library this does NOT
    // block waiting for ACK — it just writes the reset frame to Serial2 and
    // returns. The DFPlayer then re-mounts the SD card and fires its
    // OnPlaySourceOnline notification somewhere between 500 and 1500 ms
    // later. Without this reset some DFPlayer clones never announce their
    // SD card on their own.
    Serial.println("  [CP8d] reset() + pump loop() for SD-online..."); Serial.flush();
    dfPlayer->reset();

    // Settle window: pump loop() for up to SOUND_INIT_WINDOW_MS and watch
    // for SD-online. OnPlaySourceOnline sets status.cardOnline = true when
    // the DFPlayer reports its SD card is ready. 3 seconds is the window
    // Makuna's examples use after reset() — some clones take that long to
    // complete the SD-mount handshake.
    const uint32_t SOUND_INIT_WINDOW_MS = 3000;
    uint32_t tStart = millis();
    while (!status.cardOnline && (millis() - tStart) < SOUND_INIT_WINDOW_MS) {
        dfPlayer->loop();
        esp_task_wdt_reset();
        delay(20);
    }

    if (!status.cardOnline) {
        Serial.println("  [CP8d] timeout — no SD-online notification"); Serial.flush();
        Serial.println("  [CP8] DFPlayer absent / unresponsive — sound stays OFF");
        Serial.flush();
        status.errorMessage = "No DFPlayer / SD at boot";
        // Keep the Makuna object allocated so update() can keep draining
        // the UART; if the DFPlayer comes online later, OnPlaySourceOnline
        // still fires and we flip cardOnline. But we do NOT set
        // initialized = true, so no playback is attempted.
        return false;
    }
    Serial.println("  [CP8d] SD-online received"); Serial.flush();

    Serial.println("  [CP8e] setVolume + setEq"); Serial.flush();
    setVolume(DEFAULT_VOLUME);
    dfPlayer->setEq(DfMp3_Eq_Normal);
    Serial.println("  [CP8e] OK"); Serial.flush();

    clearQueue();
    status.initialized = true;
    Serial.println("  [CP8] sound controller ready (Makuna backend)");
    Serial.flush();
    return true;
}

void SoundController::end() {
    if (dfPlayer) {
        dfPlayer->stop();
        delete dfPlayer;
        dfPlayer = nullptr;
    }
    status.initialized = false;
    status.playing     = false;
}

// -------------------------------------------------------------------------
// update() — must be called every loop. dfPlayer->loop() processes the
// async reply stream from the DFPlayer and fires Mp3Notify callbacks.
// Our own queue-draining runs afterwards.
// -------------------------------------------------------------------------
void SoundController::update() {
    // Always pump loop() while the DfMp3 object exists — even if begin()
    // could not confirm SD-online during setup, the DFPlayer may be
    // inserted later at runtime and its notifications (SD online /
    // inserted / removed / errors) must still reach Mp3Notify.
    if (!dfPlayer) return;
    dfPlayer->loop();

    // Playback only when begin() reported success AND the SD is online.
    if (!status.initialized) return;

    uint32_t now = millis();
    if (!muted && queueCount > 0 && !status.playing) {
        if (now - lastSoundTime >= MIN_SOUND_INTERVAL) {
            processQueue();
        }
    }

    if (now - lastStatusCheck >= 1000) {
        lastStatusCheck = now;
    }
}

// -------------------------------------------------------------------------
// Playback
// -------------------------------------------------------------------------

bool SoundController::playSound(uint8_t track) {
    if (!status.initialized || muted) return false;
    if (track < 1) return false;
    return addToQueue(track, SOUND_CAT_CUSTOM, 5);
}

bool SoundController::playRandomFromCategory(SoundCategory category) {
    uint8_t track = getRandomTrack(category);
    if (track > 0) return addToQueue(track, category, 5);
    return false;
}

void SoundController::processQueue() {
    if (queueCount == 0 || !dfPlayer) return;

    SoundQueueItem item = soundQueue[queueHead];
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    queueCount--;

    // Tracks live in /mp3/NNNN.mp3 — Makuna helper maps the filename pattern.
    dfPlayer->playMp3FolderTrack(item.track);
    status.currentTrack = item.track;
    status.playing      = true;
    lastSoundTime       = millis();
}

bool SoundController::addToQueue(uint8_t track, SoundCategory category, uint8_t priority) {
    if (queueCount >= QUEUE_SIZE) return false;
    soundQueue[queueTail].track     = track;
    soundQueue[queueTail].timestamp = millis();
    soundQueue[queueTail].category  = category;
    soundQueue[queueTail].priority  = priority;
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount++;
    return true;
}

uint8_t SoundController::getRandomTrack(SoundCategory category) {
    uint8_t startTrack = 0, endTrack = 0;
    switch (category) {
        case SOUND_CAT_GREETING:
            startTrack = SOUND_GREET_START;   endTrack = SOUND_GREET_END;   break;
        case SOUND_CAT_NEGATIVE:
            startTrack = SOUND_NEG_START;     endTrack = SOUND_NEG_END;     break;
        case SOUND_CAT_POSITIVE:
            startTrack = SOUND_POS_START;     endTrack = SOUND_POS_END;     break;
        case SOUND_CAT_SQUEAKY:
            startTrack = SOUND_SQUEAK_START;  endTrack = SOUND_SQUEAK_END;  break;
        default: return 0;
    }
    if (startTrack > 0 && endTrack >= startTrack) {
        return random(startTrack, endTrack + 1);
    }
    return 0;
}

void SoundController::clearQueue() {
    queueHead = 0;
    queueTail = 0;
    queueCount = 0;
}

// -------------------------------------------------------------------------
// Transport controls
// -------------------------------------------------------------------------

void SoundController::stop() {
    if (!dfPlayer) return;
    dfPlayer->stop();
    status.playing = false;
    clearQueue();
}

void SoundController::pause() {
    if (!dfPlayer || !status.playing) return;
    dfPlayer->pause();
    status.playing = false;
}

void SoundController::resume() {
    if (!dfPlayer || status.playing) return;
    dfPlayer->start();
    status.playing = true;
}

void SoundController::next() {
    if (dfPlayer) dfPlayer->nextTrack();
}

void SoundController::previous() {
    if (dfPlayer) dfPlayer->prevTrack();
}

// -------------------------------------------------------------------------
// Volume / mute
// -------------------------------------------------------------------------

void SoundController::setVolume(uint8_t volume) {
    volume = constrain(volume, 0, 30);
    if (dfPlayer) dfPlayer->setVolume(volume);
    status.volume = volume;
}

void SoundController::volumeUp()   { if (status.volume < 30) setVolume(status.volume + 1); }
void SoundController::volumeDown() { if (status.volume > 0)  setVolume(status.volume - 1); }

void SoundController::setMute(bool mute) {
    muted = mute;
    if (muted) stop();
}

void SoundController::toggleMute() { setMute(!muted); }

// -------------------------------------------------------------------------
// RC channel handlers (unchanged logic)
// -------------------------------------------------------------------------

void SoundController::handleGreetingChannel(bool active) {
    if (active && !channelStates.greetingActive) {
        playRandomFromCategory(SOUND_CAT_GREETING);
    } else if (!active && channelStates.greetingActive) {
        playSound(SOUND_DEFAULT);
    }
    channelStates.greetingActive = active;
}

void SoundController::handleMoodChannel(int position) {
    static int lastPosition = 1;
    if (position != lastPosition) {
        if      (position == 0) playRandomFromCategory(SOUND_CAT_NEGATIVE);
        else if (position == 2) playRandomFromCategory(SOUND_CAT_POSITIVE);
        lastPosition = position;
    }
}

void SoundController::handleSqueakyChannel(bool active) {
    if (active && !channelStates.squeakyActive) {
        playRandomFromCategory(SOUND_CAT_SQUEAKY);
    }
    channelStates.squeakyActive = active;
}

// -------------------------------------------------------------------------
// Status / diagnostics
// -------------------------------------------------------------------------

bool SoundController::isReady() {
    return status.initialized && status.cardOnline && !muted;
}

String SoundController::getStatusString() {
    String s = "Sound Controller (Makuna):\n";
    s += "Initialized: "; s += (status.initialized ? "Yes" : "No"); s += "\n";
    s += "Card Online: "; s += (status.cardOnline  ? "Yes" : "No"); s += "\n";
    s += "Playing: ";     s += (status.playing     ? "Yes" : "No"); s += "\n";
    s += "Muted: ";       s += (muted              ? "Yes" : "No"); s += "\n";
    s += "Volume: ";      s += status.volume; s += "\n";
    s += "Queue: ";       s += queueCount; s += "/"; s += QUEUE_SIZE; s += "\n";
    if (status.errorMessage.length() > 0) {
        s += "Error: " + status.errorMessage + "\n";
    }
    return s;
}

// -------------------------------------------------------------------------
// Special effects
// -------------------------------------------------------------------------

void SoundController::playStartupSequence()  { if (isReady()) addToQueue(SOUND_STARTUP,   SOUND_CAT_SYSTEM, 10); }
void SoundController::playShutdownSequence() { if (isReady()) addToQueue(SOUND_NEG_START, SOUND_CAT_SYSTEM, 10); }
void SoundController::playErrorSound()       { if (isReady()) addToQueue(SOUND_NEG_START, SOUND_CAT_SYSTEM,  9); }
void SoundController::playSuccessSound()     { if (isReady()) addToQueue(SOUND_POS_START, SOUND_CAT_SYSTEM,  9); }

// -------------------------------------------------------------------------
// Advanced
// -------------------------------------------------------------------------

void SoundController::enableLoopMode(bool enable) {
    if (!dfPlayer) return;
    // Makuna doesn't expose a single "enableLoop()" toggle. The closest
    // equivalents are per-folder/per-track repeat modes. Leave as no-op for
    // now; re-wire once we know which repeat mode we actually want.
    (void)enable;
}

void SoundController::setEQ(uint8_t eq) {
    if (!dfPlayer || eq > 5) return;
    DfMp3_Eq mode = DfMp3_Eq_Normal;
    switch (eq) {
        case 0: mode = DfMp3_Eq_Normal;  break;
        case 1: mode = DfMp3_Eq_Pop;     break;
        case 2: mode = DfMp3_Eq_Rock;    break;
        case 3: mode = DfMp3_Eq_Jazz;    break;
        case 4: mode = DfMp3_Eq_Classic; break;
        case 5: mode = DfMp3_Eq_Bass;    break;
    }
    dfPlayer->setEq(mode);
}

void SoundController::sleep() {
    if (dfPlayer) dfPlayer->sleep();
}

void SoundController::wakeUp() {
    if (!dfPlayer) return;
    dfPlayer->reset();
    delay(200);
}
