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
    screenWidth(TFT_WIDTH),
    screenHeight(TFT_HEIGHT) {
    
    // Initialize status message
    memset(&statusMessage, 0, sizeof(statusMessage));
    memset(&telemetryBuffer, 0, sizeof(telemetryBuffer));
    memset(diagnosticsBuffer, 0, sizeof(diagnosticsBuffer));
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

// Process a chunk of the display update
bool DisplayHandler::processUpdateChunk() {
    switch (updateState) {
        case UPDATE_CLEAR:
            // Clear in chunks if needed
            tft.fillScreen(COLOR_BACKGROUND);
            updateState = UPDATE_HEADER;
            return false;
            
        case UPDATE_HEADER:
            // Draw header based on mode
            switch (currentMode) {
                case DISPLAY_MODE_TELEMETRY:
                    drawHeader("D-O TELEMETRY");
                    break;
                case DISPLAY_MODE_DIAGNOSTICS:
                    drawHeader("DIAGNOSTICS");
                    break;
                case DISPLAY_MODE_STATUS:
                    // Status messages don't need header
                    break;
            }
            updateState = UPDATE_CONTENT;
            currentLine = 0;
            return false;
            
        case UPDATE_CONTENT:
            // Draw content in chunks
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
            // Draw footer if needed
            if (currentMode == DISPLAY_MODE_TELEMETRY) {
                drawFooter("B1:Go B2:Cal B3:Disp B4:Dbg");
            }
            updateState = UPDATE_COMPLETE;
            return false;
            
        case UPDATE_COMPLETE:
            return true;  // Update finished
    }
    
    return false;
}

// Update telemetry display in chunks - OPTIMIZED LAYOUT
bool DisplayHandler::updateTelemetryChunk() {
    char buffer[32];
    
    // Smaller text size for data
    tft.setTextSize(1);
    
    // Draw a few lines per call
    for (int i = 0; i < DISPLAY_CHUNK_LINES && currentLine < 10; i++, currentLine++) {
        switch (currentLine) {
            case 0:  // Pitch
                tft.setCursor(5, 25);
                tft.setTextColor(COLOR_TEXT);
                tft.print("Pitch:");
                tft.setCursor(45, 25);
                tft.setTextColor(abs(telemetryBuffer.pitch) > 30 ? COLOR_WARNING : COLOR_SUCCESS);
                snprintf(buffer, sizeof(buffer), "%6.1f", telemetryBuffer.pitch);
                tft.print(buffer);
                break;
                
            case 1:  // Roll
                tft.setCursor(5, 37);
                tft.setTextColor(COLOR_TEXT);
                tft.print("Roll:");
                tft.setCursor(45, 37);
                tft.setTextColor(abs(telemetryBuffer.roll) > 30 ? COLOR_WARNING : COLOR_SUCCESS);
                snprintf(buffer, sizeof(buffer), "%6.1f", telemetryBuffer.roll);
                tft.print(buffer);
                break;
                
            case 2:  // Yaw
                tft.setCursor(5, 49);
                tft.setTextColor(COLOR_TEXT);
                tft.print("Yaw:");
                tft.setCursor(45, 49);
                tft.setTextColor(COLOR_TEXT);
                snprintf(buffer, sizeof(buffer), "%6.1f", telemetryBuffer.yaw);
                tft.print(buffer);
                break;
                
            case 3:  // IMU Rate
                tft.setCursor(120, 25);
                tft.setTextColor(COLOR_TEXT);
                tft.print("IMU:");
                tft.setCursor(145, 25);
                tft.setTextColor(telemetryBuffer.imuRate > 80 ? COLOR_SUCCESS : COLOR_WARNING);
                snprintf(buffer, sizeof(buffer), "%3.0fHz", telemetryBuffer.imuRate);
                tft.print(buffer);
                break;
                
            case 4:  // RC Status
                tft.setCursor(120, 37);
                tft.setTextColor(COLOR_TEXT);
                tft.print("RC:");
                tft.setCursor(140, 37);
                tft.setTextColor(telemetryBuffer.rcConnected ? COLOR_SUCCESS : COLOR_ERROR);
                tft.print(telemetryBuffer.rcConnected ? "OK" : "NO");
                break;
                
            case 5:  // Motors - in a box
                // Draw motor box
                tft.drawRect(5, 65, 230, 25, COLOR_TEXT);
                tft.setCursor(10, 70);
                tft.setTextColor(COLOR_TEXT);
                tft.print("Motors:");
                
                // Motor 1
                tft.setCursor(60, 70);
                tft.setTextColor(telemetryBuffer.motor1Speed > 0 ? COLOR_GREEN : COLOR_CYAN);
                snprintf(buffer, sizeof(buffer), "M1:%4d", telemetryBuffer.motor1Speed);
                tft.print(buffer);
                
                // Motor 2
                tft.setCursor(130, 70);
                tft.setTextColor(telemetryBuffer.motor2Speed > 0 ? COLOR_GREEN : COLOR_CYAN);
                snprintf(buffer, sizeof(buffer), "M2:%4d", telemetryBuffer.motor2Speed);
                tft.print(buffer);
                break;
                
            case 6:  // Visual balance indicator
                // Draw balance bar
                int centerX = 120;
                int barY = 100;
                int barWidth = 100;
                int barHeight = 8;
                
                // Background
                tft.fillRect(centerX - barWidth/2, barY, barWidth, barHeight, COLOR_GRID);
                
                // Center line
                tft.drawLine(centerX, barY - 2, centerX, barY + barHeight + 2, COLOR_TEXT);
                
                // Pitch indicator
                int pitchX = centerX + (telemetryBuffer.pitch * barWidth / 90);
                pitchX = constrain(pitchX, centerX - barWidth/2 + 2, centerX + barWidth/2 - 2);
                
                uint16_t barColor = abs(telemetryBuffer.pitch) > 30 ? COLOR_ERROR : 
                                   abs(telemetryBuffer.pitch) > 15 ? COLOR_WARNING : COLOR_SUCCESS;
                
                tft.fillRect(pitchX - 2, barY + 1, 4, barHeight - 2, barColor);
                break;
        }
    }
    
    if (currentLine >= 7) {
        updateState = UPDATE_FOOTER;
    }
    
    return false;
}

// Update diagnostics display in chunks - COMPACT VERSION
bool DisplayHandler::updateDiagnosticsChunk() {
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(1);
    
    // Parse diagnostics buffer and display compactly
    int y = 25;
    char* line = strtok(diagnosticsBuffer, "\n");
    while (line != NULL && y < screenHeight - 15) {
        tft.setCursor(5, y);
        tft.print(line);
        y += 10;  // Reduced line spacing
        line = strtok(NULL, "\n");
    }
    
    updateState = UPDATE_COMPLETE;
    return true;
}

// Update status message - CENTERED PROPERLY
bool DisplayHandler::updateStatusChunk() {
    tft.setTextColor(statusMessage.color);
    tft.setTextSize(1);
    
    // Calculate centered position
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(statusMessage.text, 0, 0, &x1, &y1, &w, &h);
    
    int x = (screenWidth - w) / 2;
    int y = (screenHeight - h) / 2;
    
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
    tft.setRotation(rotation);
    screenWidth = tft.width();
    screenHeight = tft.height();
}

// Set display mode
void DisplayHandler::setMode(DisplayMode mode) {
    if (currentMode != mode) {
        currentMode = mode;
        updateState = UPDATE_CLEAR;
        updateInProgress = true;
        currentLine = 0;
    }
}

// Show splash screen - OPTIMIZED
void DisplayHandler::showSplashScreen() {
    if (!initialized) return;
    
    clearScreen();
    
    // Large title
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(3);
    centerText("D-O", 30);
    
    // Version info
    tft.setTextSize(1);
    centerText("ESP32-S3 v1.1", 65);
    
    // Status
    tft.setTextSize(1);
    centerText("Initializing...", 85);
    
    // Progress bar frame
    tft.drawRect(40, 100, 160, 10, COLOR_TEXT);
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
    float cpuTemp, float imuTemp,
    uint32_t freeHeap, uint32_t uptime
) {
    if (!initialized || currentMode != DISPLAY_MODE_DIAGNOSTICS) return;
    
    // Format diagnostics to buffer - COMPACT VERSION
    snprintf(diagnosticsBuffer, sizeof(diagnosticsBuffer),
             "IMU: %s\n"
             "Sound: %s\n"
             "IMU T: %.1fC\n"
             "Free: %lu KB\n"
             "Up: %02lu:%02lu:%02lu",
             imuStatus, soundStatus, imuTemp,
             freeHeap / 1024,
             (uptime / 3600000), (uptime / 60000) % 60, (uptime / 1000) % 60);
    
    // Start update if not busy
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