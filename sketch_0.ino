#include <Arduino.h>
#include <ThingerESP32.h>
#include <WiFi.h>

#define BOARD_BAUD 115200

#define PIN_RELAY_L298N_5V   7
#define PIN_FSR_ESEN068_5V   20
#define PIN_FSR_ESEN068_DATA 17

#define PIN_BUILT_IN_LED 16
#define PIN_RGBLED_R     25
#define PIN_RGBLED_G     26
#define PIN_RGBLED_B     27

#define TNGR_USERNAME    ""
#define TNGR_DEVICE_ID   ""
#define TNGR_DEVICE_CRED ""

#define WIFI_SSID        ""
#define WIFI_PASSWORD    ""

#define AUTO_BYPASS_WARNING_AMOUNT 10
#define AUTO_BYPASS_FOOD_AMOUNT    20

enum LEDState {
    OFF,
    ON,
    BLINK
};

enum ColourIndex {
    RED,
    ORANGE,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    PURPLE,
    WHITE
};

ThingerESP32 Thing(TNGR_USERNAME, TNGR_DEVICE_ID, TNGR_DEVICE_CRED);
pson         data;

wl_status_t  wifiStatus          = WL_IDLE_STATUS;

bool     isRunningAutomatically   = false;
bool     isEmergencyHaltOn        = false;
bool     manualFeedWarning        = false;
bool     allowNeglectBypass       = false;
uint16_t autoFeedAmount_g         = 0;
uint16_t manualFeedAmount_g       = 0;
uint16_t remainingFoodThreshold_g = 0;

uint8_t  ignoredWarningAmount     = 0;

bool     isDispending             = false;
float    fedFoodRemaining_g       = 0;

const uint8_t COLIDX_RGB_VMAP[9][3] = {
    {0xFF, 0x00, 0x00},
    {0xFF, 0x1C, 0x00},
    {0xFF, 0x5E, 0x00},
    {0x2F, 0xFF, 0x00},
    {0x00, 0xFF, 0xFF},
    {0x00, 0x00, 0xFF},
    {0x80, 0x00, 0x80},
    {0xFF, 0x50, 0xAF}
};

void connectToWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void setRGBLED(
    uint8_t  state,
    uint8_t  colour               = WHITE,
    uint8_t  repeatAmount         = 0,
    uint16_t blinkUpDuration_ms   = 0,
    uint16_t blinkDownDuration_ms = 0
) {
    const uint8_t *COLOUR_RGB = COLIDX_RGB_VMAP[colour];

    if (state <= 1) {
        analogWrite(PIN_RGBLED_R, (bool) state ? COLOUR_RGB[0] : 0x00);
        analogWrite(PIN_RGBLED_G, (bool) state ? COLOUR_RGB[1] : 0x00);
        analogWrite(PIN_RGBLED_B, (bool) state ? COLOUR_RGB[2] : 0x00);

        return;
    }

    setRGBLED(OFF);
    delay(blinkDownDuration_ms);

    for (uint8_t blinked = 0; blinked < repeatAmount; blinked++) {
        setRGBLED(ON, colour);
        delay(blinkUpDuration_ms);
        setRGBLED(OFF);
        delay(blinkDownDuration_ms);
    }

    setRGBLED(OFF);
}

void refreshDeviceProperty() {
    Thing.get_property("general_property", data);

    isRunningAutomatically   = (bool)     data["is_running_automatically"];
    isEmergencyHaltOn        = (bool)     data["is_emergency_halt_on"];
    allowNeglectBypass       = (bool)     data["allow_neglect_bypass"];
    autoFeedAmount_g         = (uint16_t) data["auto_feed_amount_g"];
    manualFeedAmount_g       = (uint16_t) data["manual_feed_amount_g"];
    manualFeedWarning        = (bool)     data["manual_feed_warning"];
    remainingFoodThreshold_g = (uint16_t) data["remaining_food_threshold_g"];
}

void dumpVarSerial() {
    Serial.println("[!] NEW PROPERTY VALUES:");
    Serial.printf("    IS RUNNING AUTOMATICALLY: %s\n", isRunningAutomatically ? "YES" : "NO");
    Serial.printf("    IS EMERGENCY HALT ON:     %s\n", isEmergencyHaltOn      ? "YES" : "NO");
    Serial.printf("    ALLOW BYPASS ON NEGLECT:  %s\n", allowNeglectBypass     ? "YES" : "NO");
    Serial.printf("    AUTO FEED AMOUNT (G):     %d\n", autoFeedAmount_g);
    Serial.printf("    MANUAL FEED AMOUNT (G):   %d\n", manualFeedAmount_g);
    Serial.printf("    MANUAL FEED WARNING:      %d\n", manualFeedWarning      ? "ENABLED" : "DISABLED");
    Serial.printf("    REMAINING THRESHOLD (G):  %d\n", remainingFoodThreshold_g);
}

void setup() {
    Serial.begin(BOARD_BAUD);

    Serial.println("[/] SETTING UP PINS...");

    pinMode(PIN_RGBLED_R, OUTPUT);
    pinMode(PIN_RGBLED_G, OUTPUT);
    pinMode(PIN_RGBLED_B, OUTPUT);

    setRGBLED(ON, RED);

    Serial.println("[/] INITIALISING PROGRAM...");
    delay(1000);

    setRGBLED(OFF);
    delay(100);

    setRGBLED(ON, YELLOW);

    Serial.println("[/] ESTABLISHING WIFI CONNECTION...");

    while (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();

        setRGBLED(BLINK, YELLOW, 3, 100, 100);
        delay(1000);
    }

    setRGBLED(ON, CYAN);

    Serial.println("[/] SETTING UP HANDLERS...");

    Thing["update_signal"] << [](pson &in) {
        if (!(bool) in)
            return;

        setRGBLED(OFF);
        delay(100);
        setRGBLED(ON, CYAN);

        Serial.println("[!] NEW UPDATE SIGNAL RECEIVED");
        Serial.println("[/] GATHERING NEW PROPERTIES...");

        refreshDeviceProperty();
        setRGBLED(BLINK, CYAN, 3, 100, 100);

        dumpVarSerial();

        delay(500);
    };

    Thing["manual_feed"] << [](pson &in) {
        if (!(bool) in || isEmergencyHaltOn)
            return;

        setRGBLED(ON, GREEN);
        
        dispendFood();

        setRGBLED(BLINK, GREEN, 3, 1000, 500);

        ignoredWarningAmount = 0;
    };

    setRGBLED(OFF);

    Serial.println("[!] INTIALISATION COMPLETED!");

    setRGBLED(BLINK, GREEN, 3, 100, 100);
    setRGBLED(ON, RED);
}

void loop() {

    wifiStatus = WiFi.status();
    fedFoodRemaining_g = 0.0f;

    // If WiFi is disconnected, keep retrying (every 1s) and do nothing else -
    // until the connection is re-established.
    if (wifiStatus != WL_CONNECTED) {
        connectToWiFi();

        setRGBLED(BLINK, YELLOW, 3, 100, 100);
        delay(1000);

        return;
    }

    Thing.handle();

    // If these value stucks at its invalid state (all zeros)
    // Keep fetching the property until all the value is what it is meant to be.
    while (manualFeedAmount_g == 0 || autoFeedAmount_g == 0 || remainingFoodThreshold_g == 0) {
        refreshDeviceProperty();
        dumpVarSerial();

        setRGBLED(BLINK, RED, 3, 100, 100);
    }

    // If emergency halt is on, do nothing, resetting the loop -
    // until emergency halt is turned off.
    if (isEmergencyHaltOn) {
        setRGBLED(BLINK, RED, 1, 1000, 500);

        return;
    }

    // If not automatic mode is disabled, do nothing even tho connections are met.
    if (!isRunningAutomatically) {
        setRGBLED(ON, ORANGE);

        // Flash the light to warnn the user that the food should be refilled.
        // Only if it's enabled tho.
        if (fedFoodRemaining_g < remainingFoodThreshold_g) {
            ignoredWarningAmount += 1;

            if (manualFeedWarning) {
                setRGBLED(BLINK, ORANGE, 3, 100, 100);
                setRGBLED(ON, ORANGE);
            }

            if (allowNeglectBypass && (ignoredWarningAmount > AUTO_BYPASS_WARNING_AMOUNT || AUTO_BYPASS_FOOD_AMOUNT)) {
                setRGBLED(BLINK, GREEN, 3, 100, 100);
                setRGBLED(ON, GREEN);

                dispendFood();

                setRGBLED(OFF);
            }

            delay(10000);
        }

        return;
    }

    if (fedFoodRemaining_g < remainingFoodThreshold_g) {
        setRGBLED(ON, WHITE);

        dispendFood();

        return;
    }
}