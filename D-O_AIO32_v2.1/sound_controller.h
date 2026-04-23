/**
 * Sound Controller for D-O Droid (AIO32 v2.1)
 *
 * Uses Makuna's DFMiniMp3 library (github.com/Makuna/DFMiniMp3) instead of
 * DFRobot's DFRobotDFPlayerMini. Reason: the DFRobot library's begin()
 * blocks inside a handshake loop and crashes the ESP32-S3 if the DFPlayer
 * doesn't reply within the library default timeout. Makuna's variant is
 * non-blocking, callback-notification based, and much more robust on
 * ESP32 with HardwareSerial.
 *
 * Install via Arduino Library Manager: "DFMiniMp3" by Makuna.
 */

#ifndef SOUND_CONTROLLER_H
#define SOUND_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include <DFMiniMp3.h>

// ---------------------------------------------------------------------------
// DFMiniMp3 typedef + notification class
//
// Makuna's library uses a CRTP-style callback pattern: you define a class
// with *static* methods that are invoked from the DFMiniMp3 template on
// error, play-finished, SD-insert/remove, etc. We forward-declare the
// notify class so the typedef compiles, then declare it fully below.
// ---------------------------------------------------------------------------
class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;

class Mp3Notify {
public:
    static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action);
    static void OnError(DfMp3& mp3, uint16_t errorCode);
    static void OnPlayFinished(DfMp3& mp3, DfMp3_PlaySources source, uint16_t track);
    static void OnPlaySourceOnline(DfMp3& mp3, DfMp3_PlaySources source);
    static void OnPlaySourceInserted(DfMp3& mp3, DfMp3_PlaySources source);
    static void OnPlaySourceRemoved(DfMp3& mp3, DfMp3_PlaySources source);
};

// ---------------------------------------------------------------------------
// Public types (unchanged API for the rest of the sketch)
// ---------------------------------------------------------------------------

// Sound categories
enum SoundCategory {
    SOUND_CAT_SYSTEM,
    SOUND_CAT_GREETING,
    SOUND_CAT_NEGATIVE,
    SOUND_CAT_POSITIVE,
    SOUND_CAT_SQUEAKY,
    SOUND_CAT_CUSTOM
};

// Sound queue structure
struct SoundQueueItem {
    uint8_t track;
    uint32_t timestamp;
    SoundCategory category;
    uint8_t priority;
};

// DFPlayer status
struct DFPlayerStatus {
    bool initialized;
    bool cardOnline;
    bool playing;
    uint8_t currentTrack;
    uint8_t volume;
    uint16_t totalTracks;
    uint32_t lastError;
    String errorMessage;
};

class SoundController {
private:
    // DFPlayer object (Makuna template instantiation)
    DfMp3* dfPlayer;
    HardwareSerial* playerSerial;

    // Status
    DFPlayerStatus status;
    bool muted;

    // Sound queue
    static const uint8_t QUEUE_SIZE = 10;
    SoundQueueItem soundQueue[QUEUE_SIZE];
    uint8_t queueHead;
    uint8_t queueTail;
    uint8_t queueCount;

    // Timing
    uint32_t lastSoundTime;
    uint32_t lastStatusCheck;

    // Channel states for RC control
    struct {
        bool greetingActive;
        bool negativeActive;
        bool positiveActive;
        bool squeakyActive;
        uint32_t lastTriggerTime[4];
    } channelStates;

    // Private methods
    void processQueue();
    bool addToQueue(uint8_t track, SoundCategory category, uint8_t priority = 5);
    void clearQueue();
    uint8_t getRandomTrack(SoundCategory category);

public:
    SoundController();
    ~SoundController();

    // Initialization
    bool begin();
    void end();

    // Basic control
    bool playSound(uint8_t track);
    bool playRandomFromCategory(SoundCategory category);
    void stop();
    void pause();
    void resume();
    void next();
    void previous();

    // Volume control
    void setVolume(uint8_t volume);
    uint8_t getVolume() { return status.volume; }
    void volumeUp();
    void volumeDown();

    // Mute control
    void setMute(bool mute);
    bool isMuted() { return muted; }
    void toggleMute();

    // RC channel handlers
    void handleGreetingChannel(bool active);
    void handleMoodChannel(int position); // 0=negative, 1=neutral, 2=positive
    void handleSqueakyChannel(bool active);

    // Status
    DFPlayerStatus getStatus() { return status; }
    bool isPlaying() { return status.playing; }
    bool isReady();
    String getStatusString();

    // Update - call in main loop
    void update();

    // Special effects
    void playStartupSequence();
    void playShutdownSequence();
    void playErrorSound();
    void playSuccessSound();

    // Advanced features
    void enableLoopMode(bool enable);
    void setEQ(uint8_t eq); // 0-5: Normal/Pop/Rock/Jazz/Classic/Bass
    void sleep();
    void wakeUp();

    // Notify-class friendship — lets Mp3Notify update our status struct
    // when the library signals online/removed/error.
    friend class Mp3Notify;
};

// Global instance
extern SoundController soundController;

#endif // SOUND_CONTROLLER_H
