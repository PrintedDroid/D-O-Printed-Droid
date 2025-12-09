/**
 * Sound Controller Implementation
 * ESP32-optimized version with improved queue management
 */

#include "sound_controller.h"
#include "esp_task_wdt.h"

// Global instance
SoundController soundController;

// Constructor
SoundController::SoundController() : 
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

// Destructor
SoundController::~SoundController() {
    end();
}

// Initialize sound controller
bool SoundController::begin() {
    DEBUG_PRINTLN("Sound Controller: Initializing...");
    
    // Use Serial2 for DFPlayer communication
    playerSerial = &Serial2;
    playerSerial->begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
    
    // Initialize DFPlayer with timeout
    uint32_t startTime = millis();
    bool initialized = false;
    
    while (millis() - startTime < 3000) {  // 3 second timeout
        esp_task_wdt_reset();  
        
        if (initializeDFPlayer()) {
            initialized = true;
            break;
        }
        delay(100);
        esp_task_wdt_reset();  
    }
    
    if (!initialized) {
        DEBUG_PRINTLN("Sound Controller: DFPlayer not found - disabling sound");
        return false;
    }
    
    // Clear queue
    clearQueue();
    
    // Set default volume
    setVolume(DEFAULT_VOLUME);
    
    DEBUG_PRINTLN("Sound Controller: Initialized successfully");
    return true;
}

// Initialize DFPlayer
bool SoundController::initializeDFPlayer() {
    // Try to communicate with DFPlayer
    uint8_t retries = 5;
    
    while (retries > 0) {
        esp_task_wdt_reset();  
        
        if (dfPlayer.begin(*playerSerial, true, false)) {
            // Configure player
            dfPlayer.setTimeOut(500);
            
            // Check if SD card is online
            delay(100);
            esp_task_wdt_reset();  
            
            // Get total tracks
            status.totalTracks = dfPlayer.readFileCounts();
            if (status.totalTracks == -1) {
                status.totalTracks = 20; // Assume default
            }
            
            status.initialized = true;
            status.cardOnline = true;
            
            // Set output device
            dfPlayer.outputDevice(DFPLAYER_DEVICE_SD);
            
            // Set EQ
            dfPlayer.EQ(DFPLAYER_EQ_NORMAL);
            
            return true;
        }
        
        retries--;
        delay(500);
        esp_task_wdt_reset();  
    }
    
    return false;
}

// Main update function
void SoundController::update() {
    if (!status.initialized) return;
    
    uint32_t now = millis();
    
    // Process DFPlayer messages
    handleDFPlayerMessages();
    
    // Process sound queue
    if (!muted && queueCount > 0 && !status.playing) {
        if (now - lastSoundTime >= MIN_SOUND_INTERVAL) {
            processQueue();
        }
    }
    
    // Periodic status check
    if (now - lastStatusCheck >= 1000) {
        lastStatusCheck = now;
        
        // Check if card is still online
        if (dfPlayer.available()) {
            if (dfPlayer.readType() == DFPlayerCardRemoved) {
                status.cardOnline = false;
                status.errorMessage = "SD Card removed";
            }
        }
    }
}

// Process sound queue
void SoundController::processQueue() {
    if (queueCount == 0) return;
    
    // Get next item from queue
    SoundQueueItem item = soundQueue[queueHead];
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    queueCount--;
    
    // Play the sound
    dfPlayer.play(item.track);
    status.currentTrack = item.track;
    status.playing = true;
    lastSoundTime = millis();
    
    DEBUG_PRINTF("Playing track %d from category %d\n", item.track, item.category);
}

// Handle DFPlayer messages
void SoundController::handleDFPlayerMessages() {
    if (dfPlayer.available()) {
        uint8_t type = dfPlayer.readType();
        int value = dfPlayer.read();
        
        switch (type) {
            case DFPlayerPlayFinished:
                status.playing = false;
                DEBUG_PRINTF("Track %d finished\n", value);
                break;
                
            case DFPlayerError:
                handleError(type, value);
                break;
                
            case DFPlayerCardOnline:
                status.cardOnline = true;
                status.errorMessage = "";
                DEBUG_PRINTLN("SD Card detected");
                break;
                
            case DFPlayerCardRemoved:
                status.cardOnline = false;
                status.errorMessage = "SD Card removed";
                DEBUG_PRINTLN("SD Card removed!");
                break;
                
            case DFPlayerCardInserted:
                status.cardOnline = true;
                status.errorMessage = "";
                // Re-read total tracks
                status.totalTracks = dfPlayer.readFileCounts();
                DEBUG_PRINTLN("SD Card inserted");
                break;
                
            default:
                // Ignore other message types
                break;
        }
    }
}

// Handle errors
void SoundController::handleError(uint8_t type, int value) {
    status.lastError = millis();
    
    switch (value) {
        case Busy:
            status.errorMessage = "Card not found";
            status.cardOnline = false;
            break;
        case Sleeping:
            status.errorMessage = "Sleeping";
            break;
        case SerialWrongStack:
            status.errorMessage = "Serial error";
            break;
        case CheckSumNotMatch:
            status.errorMessage = "Checksum error";
            break;
        case FileIndexOut:
            status.errorMessage = "File not found";
            break;
        case FileMismatch:
            status.errorMessage = "File mismatch";
            break;
        default:
            status.errorMessage = "Unknown error: " + String(value);
            break;
    }
    
    DEBUG_PRINTF("DFPlayer Error: %s\n", status.errorMessage.c_str());
}

// Play a specific sound
bool SoundController::playSound(uint8_t track) {
    if (!status.initialized || !status.cardOnline || muted) {
        return false;
    }
    
    // Validate track number
    if (track < 1 || track > status.totalTracks) {
        DEBUG_PRINTF("Invalid track number: %d\n", track);
        return false;
    }
    
    // Add to queue with default priority
    return addToQueue(track, SOUND_CAT_CUSTOM, 5);
}

// Play random sound from category
bool SoundController::playRandomFromCategory(SoundCategory category) {
    uint8_t track = getRandomTrack(category);
    if (track > 0) {
        return addToQueue(track, category, 5);
    }
    return false;
}

// Add sound to queue
bool SoundController::addToQueue(uint8_t track, SoundCategory category, uint8_t priority) {
    if (queueCount >= QUEUE_SIZE) {
        // Queue full - remove lowest priority item if this has higher priority
        // For now, just reject
        DEBUG_PRINTLN("Sound queue full!");
        return false;
    }
    
    // Add to queue
    soundQueue[queueTail].track = track;
    soundQueue[queueTail].timestamp = millis();
    soundQueue[queueTail].category = category;
    soundQueue[queueTail].priority = priority;
    
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount++;
    
    return true;
}

// Get random track from category
uint8_t SoundController::getRandomTrack(SoundCategory category) {
    uint8_t startTrack = 0;
    uint8_t endTrack = 0;
    
    switch (category) {
        case SOUND_CAT_GREETING:
            startTrack = SOUND_GREET_START;
            endTrack = SOUND_GREET_END;
            break;
        case SOUND_CAT_NEGATIVE:
            startTrack = SOUND_NEG_START;
            endTrack = SOUND_NEG_END;
            break;
        case SOUND_CAT_POSITIVE:
            startTrack = SOUND_POS_START;
            endTrack = SOUND_POS_END;
            break;
        case SOUND_CAT_SQUEAKY:
            startTrack = SOUND_SQUEAK_START;
            endTrack = SOUND_SQUEAK_END;
            break;
        default:
            return 0;
    }
    
    if (startTrack > 0 && endTrack >= startTrack) {
        return random(startTrack, endTrack + 1);
    }
    
    return 0;
}

// Clear sound queue
void SoundController::clearQueue() {
    queueHead = 0;
    queueTail = 0;
    queueCount = 0;
}

// Stop playback
void SoundController::stop() {
    if (status.initialized) {
        dfPlayer.stop();
        status.playing = false;
        clearQueue();
    }
}

// Pause playback
void SoundController::pause() {
    if (status.initialized && status.playing) {
        dfPlayer.pause();
        status.playing = false;
    }
}

// Resume playback
void SoundController::resume() {
    if (status.initialized && !status.playing) {
        dfPlayer.start();
        status.playing = true;
    }
}

// Next track
void SoundController::next() {
    if (status.initialized) {
        dfPlayer.next();
    }
}

// Previous track
void SoundController::previous() {
    if (status.initialized) {
        dfPlayer.previous();
    }
}

// Set volume
void SoundController::setVolume(uint8_t volume) {
    if (status.initialized) {
        volume = constrain(volume, 0, 30);
        dfPlayer.volume(volume);
        status.volume = volume;
    }
}

// Volume up
void SoundController::volumeUp() {
    if (status.volume < 30) {
        setVolume(status.volume + 1);
    }
}

// Volume down
void SoundController::volumeDown() {
    if (status.volume > 0) {
        setVolume(status.volume - 1);
    }
}

// Set mute
void SoundController::setMute(bool mute) {
    muted = mute;
    if (muted) {
        stop();
    }
}

// Toggle mute
void SoundController::toggleMute() {
    setMute(!muted);
}

// Handle greeting channel
void SoundController::handleGreetingChannel(bool active) {
    if (active && !channelStates.greetingActive) {
        // Rising edge - play greeting
        playRandomFromCategory(SOUND_CAT_GREETING);
    } else if (!active && channelStates.greetingActive) {
        // Falling edge - play default
        playSound(SOUND_DEFAULT);
    }
    channelStates.greetingActive = active;
}

// Handle mood channel (3-position)
void SoundController::handleMoodChannel(int position) {
    static int lastPosition = 1; // Start at neutral
    
    if (position != lastPosition) {
        if (position == 0) {
            // Negative
            playRandomFromCategory(SOUND_CAT_NEGATIVE);
        } else if (position == 2) {
            // Positive
            playRandomFromCategory(SOUND_CAT_POSITIVE);
        }
        // Position 1 (neutral) plays nothing
        
        lastPosition = position;
    }
}

// Handle squeaky channel
void SoundController::handleSqueakyChannel(bool active) {
    if (active && !channelStates.squeakyActive) {
        // Rising edge - play squeak
        playRandomFromCategory(SOUND_CAT_SQUEAKY);
    }
    channelStates.squeakyActive = active;
}

// Check if ready
bool SoundController::isReady() {
    return status.initialized && status.cardOnline && !muted;
}

// Get status string
String SoundController::getStatusString() {
    String str = "Sound Controller:\n";
    str += "Initialized: " + String(status.initialized ? "Yes" : "No") + "\n";
    str += "Card Online: " + String(status.cardOnline ? "Yes" : "No") + "\n";
    str += "Playing: " + String(status.playing ? "Yes" : "No") + "\n";
    str += "Muted: " + String(muted ? "Yes" : "No") + "\n";
    str += "Volume: " + String(status.volume) + "\n";
    str += "Queue: " + String(queueCount) + "/" + String(QUEUE_SIZE) + "\n";
    
    if (status.errorMessage.length() > 0) {
        str += "Error: " + status.errorMessage + "\n";
    }
    
    return str;
}

// Play startup sequence
void SoundController::playStartupSequence() {
    if (isReady()) {
        // High priority
        addToQueue(SOUND_STARTUP, SOUND_CAT_SYSTEM, 10);
    }
}

// Play shutdown sequence
void SoundController::playShutdownSequence() {
    if (isReady()) {
        // Play a negative sound as shutdown
        addToQueue(SOUND_NEG_START, SOUND_CAT_SYSTEM, 10);
    }
}

// Play error sound
void SoundController::playErrorSound() {
    if (isReady()) {
        // Play first negative sound as error
        addToQueue(SOUND_NEG_START, SOUND_CAT_SYSTEM, 9);
    }
}

// Play success sound
void SoundController::playSuccessSound() {
    if (isReady()) {
        // Play first positive sound as success
        addToQueue(SOUND_POS_START, SOUND_CAT_SYSTEM, 9);
    }
}

// Enable loop mode
void SoundController::enableLoopMode(bool enable) {
    if (status.initialized) {
        if (enable) {
            dfPlayer.enableLoop();
        } else {
            dfPlayer.disableLoop();
        }
    }
}

// Set EQ
void SoundController::setEQ(uint8_t eq) {
    if (status.initialized && eq <= 5) {
        dfPlayer.EQ(eq);
    }
}

// Sleep
void SoundController::sleep() {
    if (status.initialized) {
        dfPlayer.sleep();
    }
}

// Wake up
void SoundController::wakeUp() {
    if (status.initialized) {
        dfPlayer.reset();
        delay(100);
        // Re-initialize
        initializeDFPlayer();
    }
}

// Clean up
void SoundController::end() {
    if (status.initialized) {
        stop();
        status.initialized = false;
    }
}