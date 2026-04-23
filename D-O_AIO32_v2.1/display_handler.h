/**
 * Display Handler for D-O Droid - OPTIMIZED VERSION
 * Non-blocking display updates for 240x135 ST7789
 */

#ifndef DISPLAY_HANDLER_H
#define DISPLAY_HANDLER_H

#include <Arduino.h>
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>

// Display timing
#define STATUS_MESSAGE_DURATION 2000  // Default status message duration
#define DISPLAY_CHUNK_LINES 5        // Lines to draw per update chunk

// Display layout constants - OPTIMIZED FOR 240x135
#define HEADER_HEIGHT 20
#define FOOTER_HEIGHT 15
#define MARGIN 2
#define LINE_HEIGHT 10
#define GRAPH_HEIGHT 30

// Color scheme - RGB565 format
#define COLOR_BACKGROUND 0x0000  // Black
#define COLOR_TEXT 0xFFFF       // White
#define COLOR_HEADER 0x001F     // Blue
#define COLOR_SUCCESS 0x07E0    // Green
#define COLOR_WARNING 0xFFE0    // Yellow
#define COLOR_ERROR 0xF800      // Red
#define COLOR_GRID 0x4208       // Dark gray
#define COLOR_GRAPH1 0x07FF     // Cyan
#define COLOR_GRAPH2 0xF81F     // Magenta

// Additional colors needed by the main sketch
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_YELLOW  0xFFE0
#define COLOR_ORANGE  0xFDA0

// TFT color aliases for compatibility
#define TFT_BLACK   COLOR_BLACK
#define TFT_WHITE   COLOR_WHITE
#define TFT_RED     COLOR_RED
#define TFT_GREEN   COLOR_GREEN
#define TFT_BLUE    COLOR_BLUE
#define TFT_CYAN    COLOR_CYAN
#define TFT_MAGENTA COLOR_MAGENTA
#define TFT_YELLOW  COLOR_YELLOW
#define TFT_ORANGE  COLOR_ORANGE

// Display modes
enum DisplayMode {
    DISPLAY_MODE_TELEMETRY,
    DISPLAY_MODE_DIAGNOSTICS,
    DISPLAY_MODE_GRAPH,
    DISPLAY_MODE_MENU,
    DISPLAY_MODE_SPLASH,
    DISPLAY_MODE_STATUS
};

// Update states for non-blocking operation
enum UpdateState {
    UPDATE_IDLE,
    UPDATE_CLEAR,
    UPDATE_HEADER,
    UPDATE_CONTENT,
    UPDATE_FOOTER,
    UPDATE_COMPLETE
};

// Telemetry data structure
struct TelemetryData {
    float pitch, roll, yaw;
    int motor1Speed, motor2Speed;
    bool rcConnected;
    float imuRate;
};

// Graph data structure
struct GraphData {
    static const uint8_t GRAPH_POINTS = 60;
    float values[GRAPH_POINTS];
    uint8_t index;
    float minValue;
    float maxValue;
    bool autoScale;
    
    GraphData() : index(0), minValue(-45), maxValue(45), autoScale(true) {
        memset(values, 0, sizeof(values));
    }
    
    void addPoint(float value);
};

class DisplayHandler {
private:
    // Display object - static allocation
    Adafruit_ST7789 tft;
    
    // Display state
    DisplayMode currentMode;
    DisplayMode previousMode;
    UpdateState updateState;
    bool initialized;
    bool backlightOn;
    uint8_t brightness;
    uint32_t lastUpdate;
    
    // Non-blocking update tracking
    uint8_t currentLine;
    uint32_t updateStartTime;
    bool updateInProgress;

    // Flicker-reduction: when true, the next draw rebuilds the whole screen
    // (fillScreen + header + static labels + values). When false, only the
    // dynamic value regions are cleared and redrawn; static labels and the
    // header stay in place. Set to true by setMode() and at boot.
    bool needsFullRedraw;

    // Previous telemetry values — used to skip redraw when a numeric field
    // hasn't changed meaningfully. Reduces flicker further.
    TelemetryData lastDrawnTelemetry;
    bool          lastDrawnTelemetryValid;

    // Diagnostics: structured per-field values (replaces the flat
    // diagnosticsBuffer[] text layout to enable label+value partial redraw).
    // Filled by queueDiagnosticsUpdate(), consumed by updateDiagnosticsChunk().
    struct {
        char     imuStatus[32];
        char     soundStatus[32];
        float    imuTemp;
        uint32_t freeHeapKB;
        uint32_t uptimeMs;
    } diag;
    // Last-drawn snapshot for change detection
    struct {
        char     imuStatus[32];
        char     soundStatus[32];
        float    imuTemp;
        uint32_t freeHeapKB;
        uint32_t uptimeSec;   // seconds resolution — filter sub-second ticks
    } lastDrawnDiag;
    bool lastDrawnDiagValid;
    
    // Status message management
    struct {
        char text[64];
        uint16_t color;
        uint32_t startTime;
        uint32_t duration;
        bool active;
    } statusMessage;
    
    // Data buffers
    TelemetryData telemetryBuffer;
    char diagnosticsBuffer[256];
    
    // Graph data
    GraphData pitchGraph;
    GraphData rollGraph;
    
    // Screen dimensions
    uint16_t screenWidth;
    uint16_t screenHeight;
    
    // Private helper methods
    void drawProgressBar(int x, int y, int w, int h, float value, float min, float max, uint16_t color);
    void drawGraph(GraphData& data, int x, int y, int w, int h, uint16_t color);
    void drawBattery(int x, int y, float voltage);
    void drawSignalStrength(int x, int y, int strength);
    bool processUpdateChunk();
    
    // Chunk update methods
    bool updateTelemetryChunk();
    bool updateDiagnosticsChunk();
    bool updateStatusChunk();
    
public:
    DisplayHandler();
    ~DisplayHandler() {}  // No dynamic allocation to clean up
    
    // Initialization
    bool begin();
    void end();
    
    // Basic display control
    void clearScreen();
    void setBacklight(bool on);
    void setBrightness(uint8_t level);
    void setRotation(uint8_t rotation);
    
    // Display modes
    void setMode(DisplayMode mode);
    DisplayMode getMode() { return currentMode; }
    
    // Non-blocking update function - call frequently from loop()
    bool update();
    
    // Splash screen
    void showSplashScreen();
    
    // Status displays with timeout
    void showStatus(const char* message, uint16_t color = COLOR_TEXT, uint32_t duration = STATUS_MESSAGE_DURATION);
    void showError(const char* error);
    void showState(const char* state, uint16_t color = COLOR_TEXT);
    
    // Queue telemetry update
    void queueTelemetryUpdate(
        float pitch, float roll, float yaw,
        int motor1Speed, int motor2Speed,
        bool rcConnected, float imuRate
    );
    
    // Queue diagnostics update
    void queueDiagnosticsUpdate(
        const char* imuStatus,
        const char* soundStatus,
        float cpuTemp, float imuTemp,
        uint32_t freeHeap, uint32_t uptime
    );
    
    // Graph mode
    void updateGraph(float pitch, float roll);
    void setGraphAutoScale(bool autoScale);
    void setGraphScale(float min, float max);
    
    // Menu system
    void showMenu(const char* items[], uint8_t itemCount, uint8_t selectedIndex);
    void updateMenuSelection(uint8_t selectedIndex);
    
    // Utility displays
    void showCalibrationProgress(float progress);
    void showBatteryWarning(float voltage);
    void showConnectionStatus(bool wifi, bool bluetooth, bool rc);
    
    // Direct access (use sparingly)
    Adafruit_ST7789* getDisplay() { return &tft; }
    
    // Drawing utilities
    void drawHeader(const char* title);
    void drawFooter(const char* text, uint16_t color = COLOR_TEXT);
    void centerText(const char* text, int y, uint16_t color = COLOR_TEXT);
    void rightAlignText(const char* text, int x, int y, uint16_t color = COLOR_TEXT);
    
    // Check if busy
    bool isBusy() { return updateInProgress; }
};

// Global instance
extern DisplayHandler displayHandler;

#endif // DISPLAY_HANDLER_H