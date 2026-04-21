/**
 * Sound Controller for D-O Droid
 * Manages DFPlayer Mini for sound effects
 * Ported from Arduino version with ESP32 optimizations
 */

#ifndef SOUND_CONTROLLER_H
#define SOUND_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "DFRobotDFPlayerMini.h"

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
    // DFPlayer object
    DFRobotDFPlayerMini dfPlayer;
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
    bool initializeDFPlayer();
    void processQueue();
    void handleDFPlayerMessages();
    void handleError(uint8_t type, int value);
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
};

// Global instance
extern SoundController soundController;

#endif // SOUND_CONTROLLER_H