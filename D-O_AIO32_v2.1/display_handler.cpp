/**
 * Display Handler Implementation - OPTIMIZED FOR 240x135
 * Non-blocking display updates with compact layout
 */

#include "display_handler.h"

// Global instance
DisplayHandler displayHandler;

// Constructor with member initializer list
DisplayHandler::DisplayHandler() :
    tft(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN),
    currentMode(DISPLAY_MODE_TELEMETRY),
    previousMode(DISPLAY_MODE_TELEMETRY),
    updateState(UPDATE_IDLE),
    initialized(false),
    backlightOn(true),
    brightness(255),
    lastUpdate(0),
    currentLine(0),
    updateStartTime(0),
    updateInProgress(false),
    needsFullRedraw(true),
    lastDrawnTelemetryValid(false),
    screenWidth(TFT_WIDTH),
    screenHeight(TFT_HEIGHT) {

    memset(&statusMessage, 0, sizeof(statusMessage));
    memset(&telemetryBuffer, 0, sizeof(telemetryBuffer));
    memset(&lastDrawnTelemetry, 0, sizeof(lastDrawnTelemetry));
    memset(diagnosticsBuffer, 0, sizeof(diagnosticsBuffer));
    memset(&diag, 0, sizeof(diag));
    memset(&lastDrawnDiag, 0, sizeof(lastDrawnDiag));
    lastDrawnDiagValid = false;
}

// Initialize display
bool DisplayHandler::begin() {
    DEBUG_PRINTLN("Display Handler: Initializing...");
    
    // Initialize display
    tft.init(TFT_HEIGHT, TFT_WIDTH);
    tft.setRotation(TFT_ROTATION);
    
    // Set backlight
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);
    
    // Clear screen
    tft.fillScreen(COLOR_BACKGROUND);
    
    // Set default text size - SMALLER for 240x135
    tft.setTextSize(1);
    
    screenWidth = tft.width();
    screenHeight = tft.height();
    
    initialized = true;
    DEBUG_PRINTLN("Display Handler: Initialized successfully");
    
    return true;
}

// Clean up
void DisplayHandler::end() {
    initialized = false;
    digitalWrite(TFT_BL_PIN, LOW);
}

// Main non-blocking update function
bool DisplayHandler::update() {
    if (!initialized) return false;
    
    uint32_t now = millis();
    
    // Handle status message timeout
    if (statusMessage.active && now - statusMessage.startTime > statusMessage.duration) {
        statusMessage.active = false;
        setMode(previousMode);  // Return to previous mode
    }
    
    // Process display updates in chunks
    if (updateInProgress) {
        if (processUpdateChunk()) {
            updateInProgress = false;
            updateState = UPDATE_IDLE;
            return true;  // Update complete
        }
    }
    
    return false;  // Update still in progress
}

// Process a chunk of the display update.
// Full redraw (needsFullRedraw=true) runs CLEAR -> HEADER -> CONTENT -> FOOTER.
// Partial redraw (needsFullRedraw=false) skips CLEAR + HEADER + FOOTER and
// calls the content chunk directly — which is responsible for erasing and
// redrawing only the value regions.
bool DisplayHandler::processUpdateChunk() {
    switch (updateState) {
        case UPDATE_CLEAR:
            if (needsFullRedraw) {
                tft.fillScreen(COLOR_BACKGROUND);
            }
            updateState = UPDATE_HEADER;
            return false;

        case UPDATE_HEADER:
            if (needsFullRedraw) {
                switch (currentMode) {
                    case DISPLAY_MODE_TELEMETRY:
                        drawHeader("D-O TELEMETRY"); break;
                    case DISPLAY_MODE_DIAGNOSTICS:
                        drawHeader("DIAGNOSTICS"); break;
                    case DISPLAY_MODE_STATUS:
                        break;  // no header for status
                }
            }
            updateState = UPDATE_CONTENT;
            currentLine = 0;
            return false;

        case UPDATE_CONTENT:
            switch (currentMode) {
                case DISPLAY_MODE_TELEMETRY:
                    return updateTelemetryChunk();
                case DISPLAY_MODE_DIAGNOSTICS:
                    return updateDiagnosticsChunk();
                case DISPLAY_MODE_STATUS:
                    return updateStatusChunk();
            }
            break;

        case UPDATE_FOOTER:
            if (needsFullRedraw && currentMode == DISPLAY_MODE_TELEMETRY) {
                drawFooter("B1:Go  B2:Cal  B3:Disp  B4:Dbg");
            }
            needsFullRedraw = false;   // subsequent updates are value-only
            updateState = UPDATE_COMPLETE;
            return false;

        case UPDATE_COMPLETE:
            return true;
    }
    
    return false;
}

// Update telemetry display — rotation-agnostic, size-2 text, partial redraw.
//
// Layout:
//   Header area (pre-drawn by processUpdateChunk)
//   Label + value rows for Pitch / Roll / Yaw / IMU-Hz / RC / M1 / M2
//   Balance indicator bar at bottom
//
// Labels are drawn once (needsFullRedraw==true) and stay on screen.
// Values are cleared (fillRect) and redrawn every call, so no flicker from
// full-screen clears but old numeric text never sticks around.
bool DisplayHandler::updateTelemetryChunk() {
    char buffer[32];
    const int TEXT_H     = 14;     // size-2 glyph height
    const int ROW_SPACING = 18;    // one row (label or value), size 2 + gap
    const int contentTop = HEADER_HEIGHT + 4;

    // Detect portrait orientation (tall/narrow). When portrait, labels
    // go ABOVE their values (stacked), because a side-by-side layout
    // would overflow the 135 px width at size 2. In landscape we keep
    // the side-by-side layout — fits fine in 240 px.
    const bool isPortrait = screenWidth < 200;
    const int  LABEL_X  = 4;
    const int  VALUE_X  = isPortrait ? 10             : (5 * 12);   // indent in portrait
    const int  VALUE_W  = isPortrait ? (screenWidth - 12) : (screenWidth - (5 * 12) - 8);

    // Row layout:
    //   landscape: label+value share one row  (18 px per row)
    //   portrait : label row + value row      (36 px per pair)
    const int ROW_STRIDE = isPortrait ? (ROW_SPACING * 2) : ROW_SPACING;

    int yPitch_label  = contentTop + 0 * ROW_STRIDE;
    int yRoll_label   = contentTop + 1 * ROW_STRIDE;
    int yYaw_label    = contentTop + 2 * ROW_STRIDE;
    int yMotors_label = contentTop + 3 * ROW_STRIDE;
    int yStat_label   = contentTop + 4 * ROW_STRIDE;

    int yPitch_value  = yPitch_label  + (isPortrait ? ROW_SPACING : 0);
    int yRoll_value   = yRoll_label   + (isPortrait ? ROW_SPACING : 0);
    int yYaw_value    = yYaw_label    + (isPortrait ? ROW_SPACING : 0);
    int yMotors_value = yMotors_label + (isPortrait ? ROW_SPACING : 0);
    int yStat_value   = yStat_label   + (isPortrait ? ROW_SPACING : 0);

    // On a full redraw: paint the static labels once
    if (needsFullRedraw) {
        tft.setTextSize(2);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(LABEL_X, yPitch_label);  tft.print("Pitch");
        tft.setCursor(LABEL_X, yRoll_label);   tft.print("Roll");
        tft.setCursor(LABEL_X, yYaw_label);    tft.print("Yaw");
        tft.setCursor(LABEL_X, yMotors_label); tft.print(isPortrait ? "Motors" : "M");
        tft.setCursor(LABEL_X, yStat_label);   tft.print(isPortrait ? "Hz/RC"  : "Hz");
    }

    auto drawValue = [&](int y, int w, uint16_t color, const char* text) {
        tft.fillRect(VALUE_X, y, w, TEXT_H, COLOR_BACKGROUND);
        tft.setCursor(VALUE_X, y);
        tft.setTextSize(2);
        tft.setTextColor(color);
        tft.print(text);
    };

    const int yPitch  = yPitch_value;
    const int yRoll   = yRoll_value;
    const int yYaw    = yYaw_value;
    const int yMotors = yMotors_value;
    const int yStat   = yStat_value;

    // Change detection: redraw a value only if it has moved meaningfully
    // compared to last frame. Thresholds are tuned per field so display
    // noise doesn't churn the pixels. Labels + layout are handled above.
    const bool force = needsFullRedraw || !lastDrawnTelemetryValid;
    const float ANGLE_EPS = 0.1f;                    // 0.1° angle jitter window
    const int   MOTOR_EPS = 2;                       // ±2 PWM step noise
    const float HZ_EPS    = 0.5f;                    // 0.5 Hz window

    // Pitch
    if (force || fabsf(telemetryBuffer.pitch - lastDrawnTelemetry.pitch) >= ANGLE_EPS) {
        uint16_t pc = (abs(telemetryBuffer.pitch) > 30) ? COLOR_WARNING : COLOR_SUCCESS;
        snprintf(buffer, sizeof(buffer), "%6.1f", telemetryBuffer.pitch);
        drawValue(yPitch, VALUE_W, pc, buffer);
    }

    // Roll
    if (force || fabsf(telemetryBuffer.roll - lastDrawnTelemetry.roll) >= ANGLE_EPS) {
        uint16_t rc = (abs(telemetryBuffer.roll) > 30) ? COLOR_WARNING : COLOR_SUCCESS;
        snprintf(buffer, sizeof(buffer), "%6.1f", telemetryBuffer.roll);
        drawValue(yRoll, VALUE_W, rc, buffer);
    }

    // Yaw
    if (force || fabsf(telemetryBuffer.yaw - lastDrawnTelemetry.yaw) >= ANGLE_EPS) {
        snprintf(buffer, sizeof(buffer), "%6.1f", telemetryBuffer.yaw);
        drawValue(yYaw, VALUE_W, COLOR_TEXT, buffer);
    }

    // Motors
    if (force ||
        abs(telemetryBuffer.motor1Speed - lastDrawnTelemetry.motor1Speed) >= MOTOR_EPS ||
        abs(telemetryBuffer.motor2Speed - lastDrawnTelemetry.motor2Speed) >= MOTOR_EPS) {
        snprintf(buffer, sizeof(buffer), "%4d/%4d",
                 telemetryBuffer.motor1Speed, telemetryBuffer.motor2Speed);
        drawValue(yMotors, VALUE_W, COLOR_CYAN, buffer);
    }

    // IMU Hz + RC status
    if (force ||
        fabsf(telemetryBuffer.imuRate - lastDrawnTelemetry.imuRate) >= HZ_EPS ||
        telemetryBuffer.rcConnected != lastDrawnTelemetry.rcConnected) {
        uint16_t hzCol = (telemetryBuffer.imuRate > 80) ? COLOR_SUCCESS : COLOR_WARNING;
        snprintf(buffer, sizeof(buffer), "%3.0f %s",
                 telemetryBuffer.imuRate, telemetryBuffer.rcConnected ? "RC" : "--");
        drawValue(yStat, VALUE_W, hzCol, buffer);
    }

    // Cache last-drawn snapshot (currently unused — placeholder for future
    // change-detection to skip redraws where the value hasn't moved enough
    // to be worth repainting).
    lastDrawnTelemetry = telemetryBuffer;
    lastDrawnTelemetryValid = true;

    updateState = UPDATE_FOOTER;
    return false;
}

// Update diagnostics display — size 2 text, label+value partial redraw.
// Same pattern as updateTelemetryChunk(): labels once on full redraw,
// values wiped+redrawn only when their field has actually changed.
bool DisplayHandler::updateDiagnosticsChunk() {
    char buffer[32];
    const int TEXT_H      = 14;
    const int ROW_SPACING = 18;
    const int contentTop  = HEADER_HEIGHT + 4;

    const bool isPortrait = screenWidth < 200;
    const int  LABEL_X    = 4;
    const int  VALUE_X    = isPortrait ? 10              : (6 * 12);
    const int  VALUE_W    = isPortrait ? (screenWidth - 12) : (screenWidth - (6 * 12) - 8);
    const int  ROW_STRIDE = isPortrait ? (ROW_SPACING * 2) : ROW_SPACING;

    int yImu_label  = contentTop + 0 * ROW_STRIDE;
    int ySnd_label  = contentTop + 1 * ROW_STRIDE;
    int yTemp_label = contentTop + 2 * ROW_STRIDE;
    int yHeap_label = contentTop + 3 * ROW_STRIDE;
    int yUp_label   = contentTop + 4 * ROW_STRIDE;

    int yImu  = yImu_label  + (isPortrait ? ROW_SPACING : 0);
    int ySnd  = ySnd_label  + (isPortrait ? ROW_SPACING : 0);
    int yTemp = yTemp_label + (isPortrait ? ROW_SPACING : 0);
    int yHeap = yHeap_label + (isPortrait ? ROW_SPACING : 0);
    int yUp   = yUp_label   + (isPortrait ? ROW_SPACING : 0);

    if (needsFullRedraw) {
        tft.setTextSize(2);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(LABEL_X, yImu_label);   tft.print("IMU");
        tft.setCursor(LABEL_X, ySnd_label);   tft.print(isPortrait ? "Sound" : "Snd");
        tft.setCursor(LABEL_X, yTemp_label);  tft.print(isPortrait ? "Temp"  : "T");
        tft.setCursor(LABEL_X, yHeap_label);  tft.print(isPortrait ? "Free"  : "Mem");
        tft.setCursor(LABEL_X, yUp_label);    tft.print(isPortrait ? "Up"    : "Up");
    }

    auto drawValue = [&](int y, int w, uint16_t color, const char* text) {
        tft.fillRect(VALUE_X, y, w, TEXT_H, COLOR_BACKGROUND);
        tft.setCursor(VALUE_X, y);
        tft.setTextSize(2);
        tft.setTextColor(color);
        tft.print(text);
    };

    const bool force = needsFullRedraw || !lastDrawnDiagValid;

    // IMU status (string compare)
    if (force || strcmp(diag.imuStatus, lastDrawnDiag.imuStatus) != 0) {
        drawValue(yImu, VALUE_W, COLOR_TEXT, diag.imuStatus);
    }

    // Sound status
    if (force || strcmp(diag.soundStatus, lastDrawnDiag.soundStatus) != 0) {
        drawValue(ySnd, VALUE_W, COLOR_TEXT, diag.soundStatus);
    }

    // IMU temperature — 0.5°C threshold to avoid flicker from micro-changes
    if (force || fabsf(diag.imuTemp - lastDrawnDiag.imuTemp) >= 0.5f) {
        snprintf(buffer, sizeof(buffer), "%.1fC", diag.imuTemp);
        drawValue(yTemp, VALUE_W, COLOR_TEXT, buffer);
    }

    // Free heap — only redraw when it changes by at least 1 KB
    if (force || diag.freeHeapKB != lastDrawnDiag.freeHeapKB) {
        snprintf(buffer, sizeof(buffer), "%luK", (unsigned long)diag.freeHeapKB);
        drawValue(yHeap, VALUE_W, COLOR_TEXT, buffer);
    }

    // Uptime — seconds resolution is enough; sub-second ticks shouldn't repaint
    uint32_t upSec = diag.uptimeMs / 1000;
    if (force || upSec != lastDrawnDiag.uptimeSec) {
        snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu",
                 (unsigned long)(upSec / 3600),
                 (unsigned long)((upSec / 60) % 60),
                 (unsigned long)(upSec % 60));
        drawValue(yUp, VALUE_W, COLOR_TEXT, buffer);
    }

    // Cache
    strncpy(lastDrawnDiag.imuStatus, diag.imuStatus, sizeof(lastDrawnDiag.imuStatus) - 1);
    lastDrawnDiag.imuStatus[sizeof(lastDrawnDiag.imuStatus) - 1] = '\0';
    strncpy(lastDrawnDiag.soundStatus, diag.soundStatus, sizeof(lastDrawnDiag.soundStatus) - 1);
    lastDrawnDiag.soundStatus[sizeof(lastDrawnDiag.soundStatus) - 1] = '\0';
    lastDrawnDiag.imuTemp    = diag.imuTemp;
    lastDrawnDiag.freeHeapKB = diag.freeHeapKB;
    lastDrawnDiag.uptimeSec  = upSec;
    lastDrawnDiagValid = true;

    updateState = UPDATE_COMPLETE;
    return true;
}

// Update status message - CENTERED PROPERLY
// Size 2 (~10x14 px glyphs) is the minimum readable scale on the 240x135
// ST7789. If a status string is long enough to wrap, fall back to size 1.
bool DisplayHandler::updateStatusChunk() {
    tft.setTextColor(statusMessage.color);
    tft.setTextSize(2);

    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(statusMessage.text, 0, 0, &x1, &y1, &w, &h);
    if ((int)w > screenWidth - 8) {
        tft.setTextSize(1);
        tft.getTextBounds(statusMessage.text, 0, 0, &x1, &y1, &w, &h);
    }

    int x = (screenWidth - (int)w) / 2;
    int y = (screenHeight - (int)h) / 2;
    if (x < 0) x = 0;
    if (y < 0) y = 0;

    tft.setCursor(x, y);
    tft.print(statusMessage.text);

    updateState = UPDATE_COMPLETE;
    return true;
}

// Clear screen
void DisplayHandler::clearScreen() {
    if (!initialized) return;
    tft.fillScreen(COLOR_BACKGROUND);
}

// Set backlight
void DisplayHandler::setBacklight(bool on) {
    if (!initialized) return;
    backlightOn = on;
    digitalWrite(TFT_BL_PIN, on ? HIGH : LOW);
}

// Set brightness (simplified)
void DisplayHandler::setBrightness(uint8_t level) {
    if (!initialized) return;
    brightness = level;
    digitalWrite(TFT_BL_PIN, level > 127 ? HIGH : LOW);
}

// Set rotation
void DisplayHandler::setRotation(uint8_t rotation) {
    if (!initialized) return;
    tft.setRotation(rotation & 0x03);
    screenWidth  = tft.width();
    screenHeight = tft.height();
    // After a rotation change everything on the old buffer is in the wrong
    // place. Wipe the panel, invalidate cached draws, and force a full
    // rebuild on the next update cycle.
    tft.fillScreen(COLOR_BACKGROUND);
    needsFullRedraw = true;
    lastDrawnTelemetryValid = false;
    lastDrawnDiagValid      = false;
    updateState = UPDATE_CLEAR;
    updateInProgress = true;
    currentLine = 0;
}

// Set display mode
void DisplayHandler::setMode(DisplayMode mode) {
    if (currentMode != mode) {
        currentMode = mode;
        updateState = UPDATE_CLEAR;
        updateInProgress = true;
        currentLine = 0;
        needsFullRedraw = true;            // new mode -> rebuild whole screen
        lastDrawnTelemetryValid = false;   // force next telemetry full draw
        lastDrawnDiagValid = false;        // force next diagnostics full draw
    }
}

// Show splash screen — bumped to larger text: on a 240x135 ST7789 the
// default size 1 (~5x7 px glyphs) is unreadable at arm's length. Size 2
// gives ~10x14 px; title stays at size 3 for emphasis.
void DisplayHandler::showSplashScreen() {
    if (!initialized) return;

    clearScreen();

    // Large title
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(3);
    centerText("D-O", 20);

    // Version info
    tft.setTextSize(2);
    centerText("AIO32 v2.1", 55);

    // Status
    tft.setTextSize(2);
    centerText("Initializing", 85);

    // Progress bar frame (full-width)
    tft.drawRect(20, 115, 200, 10, COLOR_TEXT);
}

// Show status message with timeout
void DisplayHandler::showStatus(const char* message, uint16_t color, uint32_t duration) {
    if (!initialized) return;
    
    // Save previous mode if not already showing status
    if (currentMode != DISPLAY_MODE_STATUS) {
        previousMode = currentMode;
    }
    
    // Copy message to buffer
    strncpy(statusMessage.text, message, sizeof(statusMessage.text) - 1);
    statusMessage.text[sizeof(statusMessage.text) - 1] = '\0';
    statusMessage.color = color;
    statusMessage.startTime = millis();
    statusMessage.duration = duration;
    statusMessage.active = true;
    
    // Switch to status mode
    setMode(DISPLAY_MODE_STATUS);
}

// Show error
void DisplayHandler::showError(const char* error) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "ERROR: %s", error);
    showStatus(buffer, COLOR_ERROR, 3000);  // Show errors longer
}

// Show state
void DisplayHandler::showState(const char* state, uint16_t color) {
    showStatus(state, color);
}

// Queue telemetry update
void DisplayHandler::queueTelemetryUpdate(
    float pitch, float roll, float yaw,
    int motor1Speed, int motor2Speed,
    bool rcConnected, float imuRate
) {
    if (!initialized || currentMode != DISPLAY_MODE_TELEMETRY) return;
    
    // Update buffer
    telemetryBuffer.pitch = pitch;
    telemetryBuffer.roll = roll;
    telemetryBuffer.yaw = yaw;
    telemetryBuffer.motor1Speed = motor1Speed;
    telemetryBuffer.motor2Speed = motor2Speed;
    telemetryBuffer.rcConnected = rcConnected;
    telemetryBuffer.imuRate = imuRate;
    
    // Start update if not busy
    if (!updateInProgress) {
        updateState = UPDATE_CONTENT;  // Skip clear for telemetry
        updateInProgress = true;
        currentLine = 0;
    }
}

// Queue diagnostics update
void DisplayHandler::queueDiagnosticsUpdate(
    const char* imuStatus,
    const char* soundStatus,
    float /*cpuTemp*/, float imuTemp,
    uint32_t freeHeap, uint32_t uptime
) {
    if (!initialized || currentMode != DISPLAY_MODE_DIAGNOSTICS) return;

    // Fill per-field members (replaces the flat snprintf buffer approach —
    // gives updateDiagnosticsChunk() a chance to redraw only what changed).
    strncpy(diag.imuStatus,   imuStatus   ? imuStatus   : "", sizeof(diag.imuStatus) - 1);
    diag.imuStatus[sizeof(diag.imuStatus) - 1] = '\0';
    strncpy(diag.soundStatus, soundStatus ? soundStatus : "", sizeof(diag.soundStatus) - 1);
    diag.soundStatus[sizeof(diag.soundStatus) - 1] = '\0';
    diag.imuTemp    = imuTemp;
    diag.freeHeapKB = freeHeap / 1024;
    diag.uptimeMs   = uptime;

    if (!updateInProgress) {
        updateState = UPDATE_CLEAR;
        updateInProgress = true;
        currentLine = 0;
    }
}

// Show calibration progress - COMPACT
void DisplayHandler::showCalibrationProgress(float progress) {
    if (!initialized) return;
    
    clearScreen();
    
    // Title
    tft.setTextSize(1);
    centerText("IMU CALIBRATION", 30);
    
    // Progress bar
    int barX = 40;
    int barY = 60;
    int barWidth = 160;
    int barHeight = 20;
    
    tft.drawRect(barX, barY, barWidth, barHeight, COLOR_TEXT);
    
    int fillWidth = (progress / 100.0) * (barWidth - 4);
    tft.fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4, COLOR_SUCCESS);
    
    // Percentage
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d%%", (int)progress);
    centerText(buffer, 90);
    
    // Instructions
    tft.setTextSize(1);
    centerText("Keep device still!", 110);
}

// Show battery warning
void DisplayHandler::showBatteryWarning(float voltage) {
    if (!initialized) return;
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "LOW BATTERY: %.1fV", voltage);
    showStatus(buffer, COLOR_ERROR, 5000);
}

// Show connection status - COMPACT VERSION
void DisplayHandler::showConnectionStatus(bool wifi, bool bluetooth, bool rc) {
    if (!initialized) return;
    
    int x = 5;
    int y = 5;
    
    tft.setTextSize(1);
    
    // Clear status area
    tft.fillRect(0, 0, 60, 15, COLOR_BACKGROUND);
    
    // WiFi
    tft.setTextColor(wifi ? COLOR_SUCCESS : COLOR_ERROR);
    tft.setCursor(x, y);
    tft.print("W");
    x += 15;
    
    // Bluetooth
    tft.setTextColor(bluetooth ? COLOR_SUCCESS : COLOR_ERROR);
    tft.setCursor(x, y);
    tft.print("B");
    x += 15;
    
    // RC
    tft.setTextColor(rc ? COLOR_SUCCESS : COLOR_ERROR);
    tft.setCursor(x, y);
    tft.print("R");
}

// Helper functions
void DisplayHandler::centerText(const char* text, int y, uint16_t color) {
    tft.setTextColor(color);
    
    // Get text bounds
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    
    int x = (screenWidth - w) / 2;
    tft.setCursor(x, y);
    tft.print(text);
}

void DisplayHandler::drawHeader(const char* title) {
    tft.fillRect(0, 0, screenWidth, HEADER_HEIGHT, COLOR_HEADER);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(1);
    centerText(title, 6);
}

void DisplayHandler::drawFooter(const char* text, uint16_t color) {
    int footerY = screenHeight - FOOTER_HEIGHT;
    tft.fillRect(0, footerY, screenWidth, FOOTER_HEIGHT, COLOR_BACKGROUND);
    tft.setTextSize(1);
    centerText(text, footerY + 3, color);
}

// Menu implementation - COMPACT VERSION
void DisplayHandler::showMenu(const char* items[], uint8_t itemCount, uint8_t selectedIndex) {
    if (!initialized) return;
    
    clearScreen();
    drawHeader("MENU");
    
    tft.setTextSize(1);
    int y = 25;
    int lineHeight = 12;
    
    // Show max 7 items on 135px display
    uint8_t maxVisible = 7;
    uint8_t startIndex = 0;
    
    // Scroll if needed
    if (selectedIndex >= maxVisible) {
        startIndex = selectedIndex - maxVisible + 1;
    }
    
    for (uint8_t i = startIndex; i < itemCount && i < startIndex + maxVisible; i++) {
        if (i == selectedIndex) {
            tft.fillRect(0, y - 2, screenWidth, lineHeight, COLOR_HEADER);
            tft.setTextColor(COLOR_TEXT);
        } else {
            tft.setTextColor(COLOR_TEXT);
        }
        
        tft.setCursor(10, y);
        tft.print(items[i]);
        
        y += lineHeight;
    }
}

void DisplayHandler::updateMenuSelection(uint8_t selectedIndex) {
    // Would need to store menu items and redraw
    // For now, just a placeholder
}

// Update graph data
void DisplayHandler::updateGraph(float pitch, float roll) {
    pitchGraph.addPoint(pitch);
    rollGraph.addPoint(roll);
    // Graph drawing would be implemented here
}

void DisplayHandler::setGraphAutoScale(bool autoScale) {
    pitchGraph.autoScale = autoScale;
    rollGraph.autoScale = autoScale;
}

void DisplayHandler::setGraphScale(float min, float max) {
    pitchGraph.minValue = min;
    pitchGraph.maxValue = max;
    rollGraph.minValue = min;
    rollGraph.maxValue = max;
}

void GraphData::addPoint(float value) {
    values[index] = value;
    index = (index + 1) % GRAPH_POINTS;
    
    if (autoScale) {
        // Update scale
        minValue = maxValue = values[0];
        for (int i = 1; i < GRAPH_POINTS; i++) {
            if (values[i] < minValue) minValue = values[i];
            if (values[i] > maxValue) maxValue = values[i];
        }
        // Add margin
        float range = maxValue - minValue;
        minValue -= range * 0.1;
        maxValue += range * 0.1;
    }
}

// Drawing utilities
void DisplayHandler::drawProgressBar(int x, int y, int w, int h, float value, float min, float max, uint16_t color) {
    if (!initialized) return;
    
    // Map value to bar width
    float normalized = (value - min) / (max - min);
    normalized = constrain(normalized, 0.0f, 1.0f);
    int fillWidth = normalized * (w - 2);
    
    // Draw outline
    tft.drawRect(x, y, w, h, color);
    
    // Clear inside
    tft.fillRect(x + 1, y + 1, w - 2, h - 2, COLOR_BACKGROUND);
    
    // Draw fill
    if (fillWidth > 0) {
        tft.fillRect(x + 1, y + 1, fillWidth, h - 2, color);
    }
}

void DisplayHandler::drawGraph(GraphData& data, int x, int y, int w, int h, uint16_t color) {
    if (!initialized) return;
    
    // Clear graph area
    tft.fillRect(x, y, w, h, COLOR_BACKGROUND);
    
    // Draw border
    tft.drawRect(x, y, w, h, COLOR_GRID);
    
    // Draw grid lines
    for (int i = 1; i < 4; i++) {
        int gridY = y + (h * i / 4);
        tft.drawLine(x, gridY, x + w, gridY, COLOR_GRID);
    }
    
    // Draw data points
    float scale = h / (data.maxValue - data.minValue);
    int lastX = x;
    int lastY = y + h / 2;
    
    for (int i = 0; i < GraphData::GRAPH_POINTS; i++) {
        int pointIndex = (data.index + i) % GraphData::GRAPH_POINTS;
        float value = data.values[pointIndex];
        
        int pointX = x + (i * w / GraphData::GRAPH_POINTS);
        int pointY = y + h - ((value - data.minValue) * scale);
        pointY = constrain(pointY, y, y + h - 1);
        
        if (i > 0) {
            tft.drawLine(lastX, lastY, pointX, pointY, color);
        }
        
        lastX = pointX;
        lastY = pointY;
    }
}

void DisplayHandler::rightAlignText(const char* text, int x, int y, uint16_t color) {
    tft.setTextColor(color);
    
    // Get text width
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    
    tft.setCursor(x - w, y);
    tft.print(text);
}

// Battery indicator - SMALLER
void DisplayHandler::drawBattery(int x, int y, float voltage) {
    if (!initialized) return;
    
    // Smaller battery icon
    tft.drawRect(x, y, 20, 10, COLOR_TEXT);
    tft.drawRect(x + 20, y + 3, 2, 4, COLOR_TEXT);
    
    // Battery level
    float percentage = map(voltage * 100, 300, 420, 0, 100) / 100.0f;
    percentage = constrain(percentage, 0.0f, 1.0f);
    
    uint16_t fillColor = percentage > 0.3f ? COLOR_SUCCESS : COLOR_ERROR;
    int fillWidth = percentage * 18;
    
    tft.fillRect(x + 1, y + 1, fillWidth, 8, fillColor);
}

// Signal strength - COMPACT
void DisplayHandler::drawSignalStrength(int x, int y, int strength) {
    if (!initialized) return;
    
    // Draw smaller signal bars
    for (int i = 0; i < 4; i++) {
        int barHeight = (i + 1) * 3;
        int barY = y + 12 - barHeight;
        
        uint16_t color = (i < (strength / 25)) ? COLOR_SUCCESS : COLOR_GRID;
        tft.fillRect(x + i * 4, barY, 3, barHeight, color);
    }
}