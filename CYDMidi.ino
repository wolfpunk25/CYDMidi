/*******************************************************************
 Wolfpunk MIDI Controller Main Launcher for ESP32 touchscreen displays
 Main file - handles setup, menu, and mode switching
 *******************************************************************/

#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <TFT_eSPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

// Include mode files
#include "keyboard_mode.h"
#include "sequencer_mode.h"
#include "bouncing_ball_mode.h"
#include "physics_drop_mode.h"
#include "random_generator_mode.h"
#include "xy_pad_mode.h"
#include "arpeggiator_mode.h"
#include "grid_piano_mode.h"
#include "auto_chord_mode.h"
#include "lfo_mode.h"
#include "ui_elements.h"
#include "midi_utils.h"

// Hardware setup
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
#define POWER_SWITCH_PIN 35
#define LCD_BACKLIGHT_PIN 21
#define BATTERY_ADC_PIN 34

// On-board common-anode RGB LED (active-low)
#define RGB_LED_RED 22
#define RGB_LED_GREEN 16
#define RGB_LED_BLUE 17

// Global objects
SPIClass mySpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
TFT_eSPI tft = TFT_eSPI();

// BLE MIDI globals
BLECharacteristic *pCharacteristic;
volatile bool deviceConnected = false;
volatile bool connectionStatusDirty = true;
uint8_t midiPacket[] = {0x80, 0x80, 0x00, 0x60, 0x7F};

// Touch state
TouchState touch;

// App state
AppMode currentMode = MENU;
unsigned long lastConnectionBlinkMs = 0;
bool connectionBlockVisible = true;
bool connectionStatusWasConnected = false;
bool displayReady = false;
const unsigned long CONNECTION_BLINK_INTERVAL_MS = 500;
const unsigned long BATTERY_UPDATE_INTERVAL_MS = 30000;
unsigned long lastBatteryUpdateMs = 0;
float batteryVoltage = 0.0f;
int batteryPercent = 0;
bool batteryPresent = false;

// Forward declarations
void drawMenu();
void drawConnectionStatus(bool force);
void updateBatteryReading(bool force);
void drawBatteryStatus();
void initializeRgbLed();
void setRgbLedColor(uint16_t color);
void setRgbLedForMode(AppMode mode);
void enterPowerOffMode();

// Scalable App Icon System
// To add new apps:
// 1. Add new mode to AppMode enum in common_definitions.h
// 2. Create mode header file (e.g., new_mode.h)
// 3. Include header in this file
// 4. Add to initialization, loop, and enterMode switch statements
// 5. Add entry to apps[] array below
// 6. Add graphics case to drawAppGraphics() function
// 7. Increment numApps
struct AppIcon {
  String name;
  String symbol;
  uint16_t color;
  AppMode mode;
};

#define MAX_APPS 12  // Can easily expand to 3x4 grid
AppIcon apps[] = {
  {"KEYS", "♪", 0xF800, KEYBOARD},     // Red
  {"BEATS", "♫", 0xFD00, SEQUENCER},   // Orange
  {"ZEN", "●", 0xFFE0, BOUNCING_BALL}, // Yellow
  {"DROP", "⬇", 0x07E0, PHYSICS_DROP}, // Green
  {"RNG", "※", 0x001F, RANDOM_GENERATOR}, // Blue
  {"XY PAD", "◈", 0x781F, XY_PAD},     // Purple
  {"ARP", "↗", 0xF81F, ARPEGGIATOR},   // Magenta
  {"GRID", "▣", 0x07FF, GRID_PIANO},   // Cyan
  {"CHORD", "⚘", 0xFBE0, AUTO_CHORD},  // Light Orange
  {"LFO", "", 0xAFE5, LFO}             // Light Green
};

int numApps = 10;

void initializeRgbLed() {
  // Keep every channel off while PWM is being attached.
  pinMode(RGB_LED_RED, OUTPUT);
  pinMode(RGB_LED_GREEN, OUTPUT);
  pinMode(RGB_LED_BLUE, OUTPUT);
  digitalWrite(RGB_LED_RED, HIGH);
  digitalWrite(RGB_LED_GREEN, HIGH);
  digitalWrite(RGB_LED_BLUE, HIGH);

  analogWrite(RGB_LED_RED, 255);
  analogWrite(RGB_LED_GREEN, 255);
  analogWrite(RGB_LED_BLUE, 255);
}

void setRgbLedColor(uint16_t color) {
  // Convert the UI's RGB565 colour to 8-bit PWM. This LED is common-anode,
  // so 0 is fully on and 255 is off for each channel.
  uint8_t red = ((color >> 11) & 0x1F) * 255 / 31;
  uint8_t green = ((color >> 5) & 0x3F) * 255 / 63;
  uint8_t blue = (color & 0x1F) * 255 / 31;

  analogWrite(RGB_LED_RED, 255 - red);
  analogWrite(RGB_LED_GREEN, 255 - green);
  analogWrite(RGB_LED_BLUE, 255 - blue);
}

void setRgbLedForMode(AppMode mode) {
  if (mode == MENU) {
    setRgbLedColor(THEME_ACCENT);  // Cyan on the launcher
    return;
  }

  for (int i = 0; i < numApps; i++) {
    if (apps[i].mode == mode) {
      setRgbLedColor(apps[i].color);
      return;
    }
  }
}

void enterPowerOffMode() {
  Serial.println("Power switch OFF - entering deep sleep");

  if (displayReady) {
    stopAllModes();
    tft.writecommand(0x28);  // ILI9341 display off
    delay(10);
    tft.writecommand(0x10);  // ILI9341 sleep in
  }

  digitalWrite(LCD_BACKLIGHT_PIN, LOW);

  // Detach PWM, drive the active-low RGB channels high, and hold all four
  // output pins in their off states throughout deep sleep.
  ledcDetach(RGB_LED_RED);
  ledcDetach(RGB_LED_GREEN);
  ledcDetach(RGB_LED_BLUE);
  pinMode(RGB_LED_RED, OUTPUT);
  pinMode(RGB_LED_GREEN, OUTPUT);
  pinMode(RGB_LED_BLUE, OUTPUT);
  digitalWrite(RGB_LED_RED, HIGH);
  digitalWrite(RGB_LED_GREEN, HIGH);
  digitalWrite(RGB_LED_BLUE, HIGH);

  gpio_hold_en((gpio_num_t)LCD_BACKLIGHT_PIN);
  gpio_hold_en((gpio_num_t)RGB_LED_RED);
  gpio_hold_en((gpio_num_t)RGB_LED_GREEN);
  gpio_hold_en((gpio_num_t)RGB_LED_BLUE);
  gpio_deep_sleep_hold_en();

  // GPIO35 is an RTC-capable input. Switching it back to 3.3V wakes the
  // controller and setup() starts again from a clean state.
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
  Serial.flush();
  esp_deep_sleep_start();
}

class MIDICallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      connectionStatusDirty = true;
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      connectionStatusDirty = true;
      // Stop all notes
      for (int i = 0; i < 128; i++) {
        sendMIDI(0x80, i, 0);
      }
      // Restart advertising so new connections can be made
      BLEDevice::startAdvertising();
    }
};

void setup() {
  Serial.begin(115200);

  // Release output states retained across deep sleep, initially keeping the
  // screen and RGB LED off to prevent a visible flash during startup.
  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, LOW);
  pinMode(RGB_LED_RED, OUTPUT);
  pinMode(RGB_LED_GREEN, OUTPUT);
  pinMode(RGB_LED_BLUE, OUTPUT);
  digitalWrite(RGB_LED_RED, HIGH);
  digitalWrite(RGB_LED_GREEN, HIGH);
  digitalWrite(RGB_LED_BLUE, HIGH);
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)LCD_BACKLIGHT_PIN);
  gpio_hold_dis((gpio_num_t)RGB_LED_RED);
  gpio_hold_dis((gpio_num_t)RGB_LED_GREEN);
  gpio_hold_dis((gpio_num_t)RGB_LED_BLUE);

  pinMode(POWER_SWITCH_PIN, INPUT);
  pinMode(BATTERY_ADC_PIN, INPUT);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  initializeRgbLed();

  // A short debounce prevents a switch transition from causing a false wake.
  if (digitalRead(POWER_SWITCH_PIN) == LOW) {
    delay(75);
    if (digitalRead(POWER_SWITCH_PIN) == LOW) {
      enterPowerOffMode();
    }
  }

  setRgbLedForMode(MENU);
  
  // Touch setup
  mySpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(mySpi);
  ts.setRotation(1);
  
  // Display setup
  tft.init();
  // This LCDWIKI panel uses inverted pixel polarity: without this command,
  // logical black is displayed as white and logical white as black.
  tft.invertDisplay(true);
  tft.setRotation(1);
  displayReady = true;
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
  
  // BLE MIDI Setup
  Serial.println("Initializing BLE MIDI...");
  BLEDevice::init("WOLFPUNK");
  Serial.println("BLE Device initialized");
  
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new MIDICallbacks());
  Serial.println("BLE Server created");
  
  BLEService *service = server->createService(BLEUUID(SERVICE_UUID));
  Serial.println("BLE Service created");
  
  pCharacteristic = service->createCharacteristic(
    BLEUUID(CHARACTERISTIC_UUID),
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE | 
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  pCharacteristic->addDescriptor(new BLE2902());
  service->start();
  Serial.println("BLE Service started");
  
  BLEAdvertising *advertising = server->getAdvertising();
  advertising->addServiceUUID(service->getUUID());
  BLEAdvertisementData adData;
  adData.setName("WOLFPUNK");
  adData.setCompleteServices(BLEUUID(SERVICE_UUID));
  advertising->setAdvertisementData(adData);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  advertising->start();
  Serial.println("BLE Advertising started - Device discoverable as 'WOLFPUNK'");
  
  // Initialize mode systems
  initializeKeyboardMode();
  initializeSequencerMode();
  initializeBouncingBallMode();
  initializeRandomGeneratorMode();
  initializeXYPadMode();
  initializeArpeggiatorMode();
  initializeGridPianoMode();
  initializeAutoChordMode();
  initializeLFOMode();
  
  drawMenu();
  updateStatus();
  Serial.println("MIDI Controller ready!");
}

void loop() {
  if (digitalRead(POWER_SWITCH_PIN) == LOW) {
    delay(75);
    if (digitalRead(POWER_SWITCH_PIN) == LOW) {
      enterPowerOffMode();
    }
  }

  updateTouch();
  
  switch (currentMode) {
    case MENU:
      updateBatteryReading(false);
      if (connectionStatusDirty) {
        connectionStatusDirty = false;
        drawConnectionStatus(true);
      } else {
        drawConnectionStatus(false);
      }
      if (touch.justPressed) handleMenuTouch();
      break;
    case KEYBOARD:
      handleKeyboardMode();
      break;
    case SEQUENCER:
      handleSequencerMode();
      break;
    case BOUNCING_BALL:
      handleBouncingBallMode();
      break;
    case PHYSICS_DROP:
      handlePhysicsDropMode();
      break;
    case RANDOM_GENERATOR:
      handleRandomGeneratorMode();
      break;
    case XY_PAD:
      handleXYPadMode();
      break;
    case ARPEGGIATOR:
      handleArpeggiatorMode();
      break;
    case GRID_PIANO:
      handleGridPianoMode();
      break;
    case AUTO_CHORD:
      handleAutoChordMode();
      break;
    case LFO:
      handleLFOMode();
      break;
  }
  
  delay(20);
}

void drawMenu() {
  tft.fillScreen(THEME_BG);
  
  // Header
  tft.fillRect(0, 0, 320, 50, THEME_SURFACE);
  tft.drawFastHLine(0, 50, 320, THEME_PRIMARY);
  tft.setTextColor(THEME_PRIMARY, THEME_SURFACE);
  tft.drawCentreString("WOLFPUNK MIDI", 160, 12, 4);
  tft.setTextColor(THEME_TEXT_DIM, THEME_SURFACE);
  tft.drawCentreString("Bluetooth Midi Controller", 160, 32, 2);
  
  // Version number
  tft.setTextColor(THEME_TEXT_DIM, THEME_SURFACE);
  tft.drawString("v0.1", 280, 37, 1);
  
  // Dynamic grid layout - 5 icons per row
  int iconSize = 40;
  int spacing = 8;  // Slightly smaller spacing for 5 columns
  int cols = 5;  // Always 5 icons per row
  int rows = (numApps + cols - 1) / cols;  // Calculate needed rows
  int startX = (320 - (cols * iconSize + (cols-1) * spacing)) / 2;
  int startY = 65;
  
  // Connection status
  drawConnectionStatus(true);
  updateBatteryReading(true);
  
  for (int i = 0; i < numApps; i++) {
    int col = i % cols;
    int row = i / cols;
    int x = startX + col * (iconSize + spacing);
    int y = startY + row * (iconSize + spacing + 15);
    
    // App icon background
    uint16_t iconColor = apps[i].color;
    
    tft.fillRoundRect(x, y, iconSize, iconSize, 8, iconColor);
    tft.drawRoundRect(x, y, iconSize, iconSize, 8, THEME_TEXT);
    
    // Draw app-specific graphics
    drawAppGraphics(apps[i].mode, x, y, iconSize);
    
    // Icon symbol
    tft.setTextColor(THEME_BG, iconColor);
    tft.drawCentreString(apps[i].symbol, x + iconSize/2, y + iconSize/2 - 8, 2);
    
    // App name
    tft.setTextColor(THEME_TEXT, THEME_BG);
    tft.drawCentreString(apps[i].name, x + iconSize/2, y + iconSize + 5, 1);
  }
}

int batteryPercentageFromVoltage(float voltage) {
  // Approximate resting-voltage curve for a single-cell LiPo. Readings while
  // the controller is busy or charging can move by a few percent.
  const float volts[] = {3.30f, 3.50f, 3.60f, 3.70f, 3.75f,
                         3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
  const int percents[] = {0, 10, 20, 35, 50, 60, 75, 85, 95, 100};
  const int points = sizeof(volts) / sizeof(volts[0]);

  if (voltage <= volts[0]) return 0;
  if (voltage >= volts[points - 1]) return 100;

  for (int i = 1; i < points; i++) {
    if (voltage <= volts[i]) {
      float fraction = (voltage - volts[i - 1]) / (volts[i] - volts[i - 1]);
      return round(percents[i - 1] + fraction * (percents[i] - percents[i - 1]));
    }
  }
  return 100;
}

void updateBatteryReading(bool force) {
  unsigned long now = millis();
  if (!force && now - lastBatteryUpdateMs < BATTERY_UPDATE_INTERVAL_MS) return;

  // The board feeds half the battery voltage to GPIO34 through two 100K
  // resistors. Average several calibrated readings to keep the display steady.
  uint32_t millivoltsTotal = 0;
  const int sampleCount = 16;
  for (int i = 0; i < sampleCount; i++) {
    millivoltsTotal += analogReadMilliVolts(BATTERY_ADC_PIN);
    delay(2);
  }

  batteryVoltage = (millivoltsTotal / (float)sampleCount) * 2.0f / 1000.0f;
  batteryPresent = batteryVoltage >= 2.5f;
  batteryPercent = batteryPresent ? batteryPercentageFromVoltage(batteryVoltage) : 0;
  lastBatteryUpdateMs = now;

  if (currentMode == MENU) drawBatteryStatus();
}

void drawBatteryStatus() {
  const int batteryY = 230;
  tft.fillRect(0, batteryY, 320, 10, THEME_BG);
  tft.setTextColor(THEME_TEXT_DIM, THEME_BG);

  if (!batteryPresent) {
    tft.drawCentreString("BATTERY NOT DETECTED", 160, batteryY, 1);
    return;
  }

  char label[24];
  snprintf(label, sizeof(label), "BAT %.2fV  %d%%", batteryVoltage, batteryPercent);
  tft.drawCentreString(label, 160, batteryY, 1);
}

void drawConnectionStatus(bool force) {
  const int statusY = 210;
  const int statusHeight = 18;
  const char* label = "CONNECT TO BLUETOOTH";
  const int blockSize = 11;
  const int blockGap = 5;
  int labelWidth = tft.textWidth(label, 2);
  int startX = (320 - labelWidth - blockGap - blockSize) / 2;
  int blockX = startX + labelWidth + blockGap;
  unsigned long now = millis();

  // BLE callbacks run on a different ESP32 task. They only mark this display
  // state as dirty; every TFT write happens here in the main loop.
  if (!force && deviceConnected != connectionStatusWasConnected) {
    force = true;
  }

  if (force) {
    tft.fillRect(0, statusY, 320, statusHeight, THEME_BG);
    tft.fillRect(286, 53, 26, 18, THEME_BG);
    tft.setTextColor(THEME_ACCENT, THEME_BG);

    if (deviceConnected) {
      tft.drawCentreString("CONNECTED", 160, statusY, 2);
      tft.fillCircle(299, 62, 4, THEME_ACCENT);
    } else {
      tft.drawString(label, startX, statusY, 2);
      connectionBlockVisible = true;
      tft.fillRect(blockX, statusY + 2, blockSize, blockSize, THEME_ACCENT);
      tft.drawCircle(299, 62, 4, THEME_ACCENT);
      lastConnectionBlinkMs = now;
    }

    connectionStatusWasConnected = deviceConnected;
    return;
  }

  if (deviceConnected || now - lastConnectionBlinkMs < CONNECTION_BLINK_INTERVAL_MS) {
    return;
  }

  connectionBlockVisible = !connectionBlockVisible;
  lastConnectionBlinkMs = now;

  // Blink by changing only the 11x11 block. Repainting the full line caused
  // unnecessary SPI traffic and visible flicker on this panel.
  tft.fillRect(blockX, statusY + 2, blockSize, blockSize,
               connectionBlockVisible ? THEME_ACCENT : THEME_BG);
}

void drawAppGraphics(AppMode mode, int x, int y, int iconSize) {
  switch (mode) {
    case KEYBOARD: // KEYS - piano keys
      {
        int keyWidth = 4;
        int totalWidth = 5 * keyWidth + 4 * 1; // 5 keys + 4 gaps
        int startX = x + (iconSize - totalWidth) / 2;
        for (int i = 0; i < 5; i++) {
          tft.fillRect(startX + i*5, y + iconSize/2 - 6, keyWidth, 12, THEME_BG);
        }
      }
      break;
    case SEQUENCER: // BEATS - grid pattern
      {
        int gridW = 4, gridH = 4, gapX = 2, gapY = 2;
        int totalW = 4 * gridW + 3 * gapX;
        int totalH = 3 * gridH + 2 * gapY;
        int startX = x + (iconSize - totalW) / 2;
        int startY = y + (iconSize - totalH) / 2;
        for (int r = 0; r < 3; r++) {
          for (int c = 0; c < 4; c++) {
            tft.fillRect(startX + c*(gridW+gapX), startY + r*(gridH+gapY), gridW, gridH, THEME_BG);
          }
        }
      }
      break;
    case BOUNCING_BALL: // ZEN - circle with dots
      {
        int centerX = x + iconSize/2;
        int centerY = y + iconSize/2;
        tft.drawCircle(centerX, centerY, 12, THEME_BG);
        tft.fillCircle(centerX - 6, centerY - 4, 2, THEME_BG);
        tft.fillCircle(centerX + 5, centerY + 2, 2, THEME_BG);
        tft.fillCircle(centerX - 2, centerY + 6, 2, THEME_BG);
      }
      break;
    case PHYSICS_DROP: // DROP - balls falling on platforms
      {
        int centerX = x + iconSize/2;
        int centerY = y + iconSize/2;
        // Draw platforms
        tft.fillRect(centerX - 10, centerY + 8, 8, 2, THEME_BG);
        tft.fillRect(centerX + 4, centerY + 4, 6, 2, THEME_BG);
        // Draw falling balls
        tft.fillCircle(centerX - 6, centerY - 8, 2, THEME_BG);
        tft.fillCircle(centerX + 2, centerY - 4, 2, THEME_BG);
        tft.fillCircle(centerX + 8, centerY, 2, THEME_BG);
      }
      break;
    case RANDOM_GENERATOR: // RNG - random dots
      {
        int centerX = x + iconSize/2;
        int centerY = y + iconSize/2;
        tft.fillCircle(centerX - 8, centerY - 6, 2, THEME_BG);
        tft.fillCircle(centerX - 1, centerY - 3, 2, THEME_BG);
        tft.fillCircle(centerX + 7, centerY + 1, 2, THEME_BG);
        tft.fillCircle(centerX - 4, centerY + 6, 2, THEME_BG);
      }
      break;
    case XY_PAD: // XY PAD - crosshairs
      {
        int centerX = x + iconSize/2;
        int centerY = y + iconSize/2;
        int crossSize = 14;
        tft.drawFastHLine(centerX - crossSize/2, centerY, crossSize, THEME_BG);
        tft.drawFastVLine(centerX, centerY - crossSize/2, crossSize, THEME_BG);
        tft.fillCircle(centerX, centerY, 3, THEME_BG);
      }
      break;
    case ARPEGGIATOR: // ARP - ascending notes
      {
        int centerX = x + iconSize/2;
        int centerY = y + iconSize/2;
        for (int i = 0; i < 4; i++) {
          tft.fillCircle(centerX - 7 + i*5, centerY + 5 - i*3, 2, THEME_BG);
        }
      }
      break;
    case GRID_PIANO: // GRID - grid pattern
      {
        int cellW = 5, cellH = 4, gapX = 1, gapY = 2;
        int totalW = 4 * cellW + 3 * gapX;
        int totalH = 3 * cellH + 2 * gapY;
        int startX = x + (iconSize - totalW) / 2;
        int startY = y + (iconSize - totalH) / 2;
        for (int r = 0; r < 3; r++) {
          for (int c = 0; c < 4; c++) {
            tft.drawRect(startX + c*(cellW+gapX), startY + r*(cellH+gapY), cellW, cellH, THEME_BG);
          }
        }
      }
      break;
    case AUTO_CHORD: // CHORD - stacked notes
      {
        int centerX = x + iconSize/2;
        int centerY = y + iconSize/2;
        int lineWidth = 14;
        tft.fillRect(centerX - lineWidth/2, centerY + 4, lineWidth, 2, THEME_BG);
        tft.fillRect(centerX - lineWidth/2, centerY, lineWidth, 2, THEME_BG);
        tft.fillRect(centerX - lineWidth/2, centerY - 4, lineWidth, 2, THEME_BG);
      }
      break;
    case LFO: // LFO - simple sine wave line
      {
        int centerX = x + iconSize/2;
        int centerY = y + iconSize/2;
        
        // Draw sine wave as connected line segments
        int lastX = centerX - 15;
        int lastY = centerY;
        
        for (int i = 1; i <= 15; i++) {
          int px = centerX - 15 + i * 2;
          float angle = (i * 3.14159) / 4.0; // One and a half cycles
          int py = centerY + (int)(6 * sin(angle));
          
          // Draw line from last point to current point
          tft.drawLine(lastX, lastY, px, py, THEME_BG);
          
          lastX = px;
          lastY = py;
        }
      }
      break;
  }
}

void handleMenuTouch() {
  int iconSize = 40;
  int spacing = 8;  // Matching the drawMenu spacing
  int cols = 5;     // Always 5 icons per row
  int startX = (320 - (cols * iconSize + (cols-1) * spacing)) / 2;
  int startY = 65;
  
  for (int i = 0; i < numApps; i++) {
    int col = i % cols;
    int row = i / cols;
    int x = startX + col * (iconSize + spacing);
    int y = startY + row * (iconSize + spacing + 15);
    
    if (isButtonPressed(x, y, iconSize, iconSize)) {
      enterMode(apps[i].mode);
      return;
    }
  }
}

void enterMode(AppMode mode) {
  currentMode = mode;
  setRgbLedForMode(mode);
  switch (mode) {
    case KEYBOARD:
      drawKeyboardMode();
      break;
    case SEQUENCER:
      drawSequencerMode();
      break;
    case BOUNCING_BALL:
      drawBouncingBallMode();
      break;
    case PHYSICS_DROP:
      drawPhysicsDropMode();
      break;
    case RANDOM_GENERATOR:
      drawRandomGeneratorMode();
      break;
    case XY_PAD:
      drawXYPadMode();
      break;
    case ARPEGGIATOR:
      drawArpeggiatorMode();
      break;
    case GRID_PIANO:
      drawGridPianoMode();
      break;
    case AUTO_CHORD:
      drawAutoChordMode();
      break;
    case LFO:
      drawLFOMode();
      break;
  }
  updateStatus();
}

void exitToMenu() {
  currentMode = MENU;
  setRgbLedForMode(MENU);
  stopAllModes();
  delay(50);
  drawMenu();
  updateStatus();
}
