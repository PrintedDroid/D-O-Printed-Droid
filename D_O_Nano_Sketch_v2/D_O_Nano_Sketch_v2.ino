/********************************************************************************
 * PROJECT: D-O Droid Sound Controller
 * AUTHOR:  Optimized version based on Printed-Droid.com design
 * DATE:    2025/06
 * VERSION: 2.0 (Performance Optimized)
 *
 * PURPOSE:
 * Controls a DFPlayer Mini MP3 player for D-O droid sound effects using a 
 * 4-channel RC receiver. This version includes significant performance 
 * improvements while maintaining hardware compatibility with existing PCB design.
 *
 * KEY IMPROVEMENTS IN 23.0:
 * - Non-blocking sequential PWM reading
 * - Intelligent sound queue management
 * - Hysteresis-based state detection
 * - Minimal serial communication in production mode
 * - Enhanced error handling without blocking
 * - Configurable parameters for easy tuning
 *
 *-------------------------------------------------------------------------------
 *
 * HARDWARE REQUIREMENTS:
 * 1. Arduino Nano (or compatible)
 * 2. DFPlayer Mini MP3 Player
 * 3. Micro SD Card (FAT16 or FAT32 formatted)
 * 4. 4-Channel RC Receiver
 * 5. Speaker (8 ohm, <3W)
 * 6. 1k Ohm Resistor (on TX line)
 *
 *-------------------------------------------------------------------------------
 *
 * WIRING (Fixed by PCB design):
 * Arduino Nano      <-->  DFPlayer Mini
 * 5V                <-->  VCC (Pin 1)
 * GND               <-->  GND (Pin 7)
 * RX (Pin D0)       <-->  TX (Pin 3)
 * TX (Pin D1)       <-->  1k Resistor <--> RX (Pin 2)
 *
 * Arduino Nano      <-->  RC Receiver
 * Pin D11           <-->  Channel 1 (Mute)
 * Pin D10           <-->  Channel 2 (Greetings)
 * Pin D12           <-->  Channel 3 (Neg/Pos)
 * Pin D9            <-->  Channel 4 (Squeaky)
 *
 *-------------------------------------------------------------------------------
 *
 * SD CARD FILE STRUCTURE:
 * /mp3/
 *   0001.mp3 - Startup sound ("battery charged")
 *   0002.mp3 - Default sound ("I am D-O")
 *   0003.mp3 to 0005.mp3 - Greeting sounds
 *   0006.mp3 to 0009.mp3 - Negative sounds
 *   0010.mp3 to 0014.mp3 - Positive sounds
 *   0015.mp3 to 0020.mp3 - Squeaky wheel sounds
 *
 ********************************************************************************/

#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// ============================================================================
// CONFIGURATION SECTION - Adjust these values for your specific setup
// ============================================================================

// Debug mode: Set to 1 for serial debugging, 0 for production
#define DEBUG_MODE 0

// Volume setting (0-30)
const uint8_t DEFAULT_VOLUME = 25;  // Not maxed out to prevent distortion

// PWM thresholds for RC channels (in microseconds)
const int PWM_THRESHOLD_LOW = 1300;   // Below this = low position
const int PWM_THRESHOLD_MID = 1500;   // Center position
const int PWM_THRESHOLD_HIGH = 1700;  // Above this = high position
const int PWM_HYSTERESIS = 50;        // Noise margin for stable switching

// Timing constants (in milliseconds)
const unsigned long PWM_TIMEOUT = 5000;        // Timeout per channel read
const unsigned long MIN_SOUND_INTERVAL = 400;  // Minimum time between sounds
const unsigned long STARTUP_DELAY = 2000;      // Wait after startup sound
const unsigned long DEBOUNCE_DELAY = 50;       // Switch debounce time

// Sound file ranges (track numbers on SD card)
const uint8_t SOUND_STARTUP = 1;
const uint8_t SOUND_DEFAULT = 2;
const uint8_t SOUND_GREET_START = 3;
const uint8_t SOUND_GREET_END = 5;
const uint8_t SOUND_NEG_START = 6;
const uint8_t SOUND_NEG_END = 9;
const uint8_t SOUND_POS_START = 10;
const uint8_t SOUND_POS_END = 14;
const uint8_t SOUND_SQUEAK_START = 15;
const uint8_t SOUND_SQUEAK_END = 20;

// ============================================================================
// DEBUG MACROS - Conditional compilation for debug messages
// ============================================================================

#if DEBUG_MODE
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINT_HEX(x) Serial.print(x, HEX)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT_HEX(x)
#endif

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// DFPlayer instance
DFRobotDFPlayerMini myDFPlayer;

// Pin definitions for RC channels
const uint8_t PIN_CH1_MUTE = 11;
const uint8_t PIN_CH2_GREET = 10;
const uint8_t PIN_CH3_MOOD = 12;
const uint8_t PIN_CH4_SQUEAK = 9;

// PWM reading state machine
enum ReadState { 
  READ_CH1, 
  READ_CH2, 
  READ_CH3, 
  READ_CH4,
  READ_COMPLETE 
};
ReadState currentReadState = READ_CH1;

// Current PWM values
int pwmValues[4] = {0, 0, 0, 0};

// Previous states for edge detection
int previousStates[4] = {0, 0, 0, 0};

// Timing variables
unsigned long lastSoundTime = 0;
unsigned long lastDebounceTime = 0;
unsigned long startupTime = 0;

// Sound system state
bool systemMuted = false;
bool startupComplete = false;

// Channel state tracking with hysteresis
struct ChannelState {
  int lastRawValue;
  int currentPosition;  // 0=low, 1=mid, 2=high
  bool hasChanged;
};
ChannelState channels[4] = {{0,0,false}, {0,0,false}, {0,0,false}, {0,0,false}};

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
  // Initialize debug serial if enabled
  DEBUG_BEGIN(9600);
  DEBUG_PRINTLN(F("D-O Sound Controller v3.0 Starting..."));
  
  // Configure RC input pins
  pinMode(PIN_CH1_MUTE, INPUT);
  pinMode(PIN_CH2_GREET, INPUT);
  pinMode(PIN_CH3_MOOD, INPUT);
  pinMode(PIN_CH4_SQUEAK, INPUT);
  
  // Initialize DFPlayer
  if (!initializeDFPlayer()) {
    // Critical error - halt execution
    DEBUG_PRINTLN(F("DFPlayer initialization failed!"));
    while(true) { 
      delay(1000); 
    }
  }
  
  // Play startup sound
  playStartupSequence();
  
  DEBUG_PRINTLN(F("Setup complete. Entering main loop."));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Read one RC channel per loop iteration (non-blocking)
  readNextChannel();
  
  // Process sound triggers only after all channels are read
  if (currentReadState == READ_COMPLETE) {
    processSoundTriggers();
    currentReadState = READ_CH1;  // Reset for next cycle
  }
  
  // Handle any DFPlayer messages (non-blocking)
  processDFPlayerMessages();
  
  // Small delay to prevent CPU hogging
  delay(1);
}

// ============================================================================
// RC CHANNEL READING FUNCTIONS
// ============================================================================

/**
 * Reads the next RC channel in sequence (non-blocking)
 */
void readNextChannel() {
  int pinToRead;
  int channelIndex;
  
  // Determine which channel to read
  switch(currentReadState) {
    case READ_CH1:
      pinToRead = PIN_CH1_MUTE;
      channelIndex = 0;
      break;
    case READ_CH2:
      pinToRead = PIN_CH2_GREET;
      channelIndex = 1;
      break;
    case READ_CH3:
      pinToRead = PIN_CH3_MOOD;
      channelIndex = 2;
      break;
    case READ_CH4:
      pinToRead = PIN_CH4_SQUEAK;
      channelIndex = 3;
      break;
    default:
      return;  // Nothing to read
  }
  
  // Read PWM value with short timeout
  int rawValue = pulseIn(pinToRead, HIGH, PWM_TIMEOUT);
  
  // Store raw value
  pwmValues[channelIndex] = rawValue;
  
  // Update channel state with hysteresis
  updateChannelState(channelIndex, rawValue);
  
  // Move to next channel
  currentReadState = (ReadState)((int)currentReadState + 1);
}

/**
 * Updates channel state with hysteresis to prevent jitter
 */
void updateChannelState(int channel, int rawValue) {
  ChannelState* ch = &channels[channel];
  int oldPosition = ch->currentPosition;
  
  // Determine new position with hysteresis
  if (channel == 2) {  // Channel 3 is 3-position
    if (rawValue > PWM_THRESHOLD_HIGH + PWM_HYSTERESIS) {
      ch->currentPosition = 2;  // High
    } else if (rawValue < PWM_THRESHOLD_LOW - PWM_HYSTERESIS) {
      ch->currentPosition = 0;  // Low  
    } else if (rawValue > PWM_THRESHOLD_MID - PWM_HYSTERESIS && 
               rawValue < PWM_THRESHOLD_MID + PWM_HYSTERESIS) {
      ch->currentPosition = 1;  // Mid
    }
    // If in transition zone, keep previous position
  } else {  // Other channels are 2-position
    if (rawValue > PWM_THRESHOLD_MID + PWM_HYSTERESIS && ch->currentPosition == 0) {
      ch->currentPosition = 1;
    } else if (rawValue < PWM_THRESHOLD_MID - PWM_HYSTERESIS && ch->currentPosition == 1) {
      ch->currentPosition = 0;
    }
  }
  
  // Check if position changed
  ch->hasChanged = (oldPosition != ch->currentPosition);
  ch->lastRawValue = rawValue;
}

// ============================================================================
// SOUND CONTROL FUNCTIONS
// ============================================================================

/**
 * Process sound triggers based on channel states
 */
void processSoundTriggers() {
  // Skip if startup not complete
  if (!startupComplete) {
    if (millis() - startupTime > STARTUP_DELAY) {
      startupComplete = true;
      DEBUG_PRINTLN(F("Startup complete, ready for input."));
    } else {
      return;
    }
  }
  
  // Channel 1: Mute control (special handling)
  if (channels[0].hasChanged) {
    systemMuted = (channels[0].currentPosition == 1);
    if (!systemMuted) {
      // Unmuted - play default sound
      queueSound(SOUND_DEFAULT);
      DEBUG_PRINTLN(F("System unmuted"));
    } else {
      DEBUG_PRINTLN(F("System muted"));
    }
    channels[0].hasChanged = false;
  }
  
  // Skip other sounds if muted
  if (systemMuted) {
    // Clear all change flags
    for (int i = 1; i < 4; i++) {
      channels[i].hasChanged = false;
    }
    return;
  }
  
  // Channel 2: Greetings vs Default
  if (channels[1].hasChanged) {
    if (channels[1].currentPosition == 1) {
      // High - play random greeting
      uint8_t track = random(SOUND_GREET_START, SOUND_GREET_END + 1);
      queueSound(track);
      DEBUG_PRINT(F("Playing greeting: "));
      DEBUG_PRINTLN(track);
    } else {
      // Low - play default sound
      queueSound(SOUND_DEFAULT);
      DEBUG_PRINTLN(F("Playing default sound"));
    }
    channels[1].hasChanged = false;
  }
  
  // Channel 3: Mood (3-position)
  if (channels[2].hasChanged) {
    if (channels[2].currentPosition == 2) {
      // High - positive sounds
      uint8_t track = random(SOUND_POS_START, SOUND_POS_END + 1);
      queueSound(track);
      DEBUG_PRINT(F("Playing positive: "));
      DEBUG_PRINTLN(track);
    } else if (channels[2].currentPosition == 0) {
      // Low - negative sounds
      uint8_t track = random(SOUND_NEG_START, SOUND_NEG_END + 1);
      queueSound(track);
      DEBUG_PRINT(F("Playing negative: "));
      DEBUG_PRINTLN(track);
    }
    // Mid position = no sound
    channels[2].hasChanged = false;
  }
  
  // Channel 4: Squeaky wheel
  if (channels[3].hasChanged) {
    if (channels[3].currentPosition == 1) {
      // High - play random squeak
      uint8_t track = random(SOUND_SQUEAK_START, SOUND_SQUEAK_END + 1);
      queueSound(track);
      DEBUG_PRINT(F("Playing squeak: "));
      DEBUG_PRINTLN(track);
    }
    channels[3].hasChanged = false;
  }
}

/**
 * Queue a sound for playback with interval checking
 */
void queueSound(uint8_t track) {
  unsigned long currentTime = millis();
  
  // Check if enough time has passed since last sound
  if (currentTime - lastSoundTime >= MIN_SOUND_INTERVAL) {
    myDFPlayer.play(track);
    lastSoundTime = currentTime;
  } else {
    DEBUG_PRINTLN(F("Sound skipped - too soon"));
  }
}

// ============================================================================
// DFPLAYER FUNCTIONS
// ============================================================================

/**
 * Initialize the DFPlayer Mini
 */
bool initializeDFPlayer() {
  // Start serial for DFPlayer
  #if !DEBUG_MODE
    Serial.begin(9600);
  #endif
  
  // Try to initialize with timeout
  unsigned long startTime = millis();
  while (!myDFPlayer.begin(Serial, true, false)) {
    if (millis() - startTime > 5000) {
      return false;  // Timeout
    }
    delay(100);
  }
  
  // Configure player
  myDFPlayer.setTimeOut(500);
  myDFPlayer.volume(DEFAULT_VOLUME);
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  
  DEBUG_PRINTLN(F("DFPlayer initialized successfully"));
  return true;
}

/**
 * Play startup sequence
 */
void playStartupSequence() {
  delay(100);  // Let DFPlayer settle
  myDFPlayer.play(SOUND_STARTUP);
  startupTime = millis();
  lastSoundTime = startupTime;
  DEBUG_PRINTLN(F("Playing startup sound"));
}

/**
 * Process incoming messages from DFPlayer (non-blocking)
 */
void processDFPlayerMessages() {
  if (myDFPlayer.available()) {
    // Read message type and value
    uint8_t type = myDFPlayer.readType();
    int value = myDFPlayer.read();
    
    // Only handle critical messages
    switch(type) {
      case DFPlayerPlayFinished:
        DEBUG_PRINT(F("Finished track: "));
        DEBUG_PRINTLN(value);
        break;
        
      case DFPlayerError:
        handleDFPlayerError(value);
        break;
        
      case DFPlayerCardRemoved:
        DEBUG_PRINTLN(F("ERROR: SD Card removed!"));
        break;
        
      case DFPlayerCardOnline:
        DEBUG_PRINTLN(F("SD Card detected"));
        break;
        
      default:
        // Ignore other message types
        break;
    }
  }
}

/**
 * Handle DFPlayer errors
 */
void handleDFPlayerError(int errorCode) {
  DEBUG_PRINT(F("DFPlayer Error: "));
  
  switch(errorCode) {
    case FileMismatch:
      DEBUG_PRINTLN(F("File not found"));
      break;
    case Busy:
      DEBUG_PRINTLN(F("Card not found"));
      break;
    default:
      DEBUG_PRINT(F("Code "));
      DEBUG_PRINTLN(errorCode);
      break;
  }
}