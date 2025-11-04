#include <Arduino.h>
#include <ThingerESP32.h>
#include <WiFi.h>

constexpr uint32_t BOARD_BAUD = 115200;

constexpr uint8_t PIN_RELAY_L298N_5V   = 7;
constexpr uint8_t PIN_FSR_ESEN068_5V   = 20;
constexpr uint8_t PIN_FSR_ESEN068_DATA = 17;

constexpr uint8_t PIN_BUILT_IN_LED = 16;
constexpr uint8_t PIN_RGBLED_R     = 25;
constexpr uint8_t PIN_RGBLED_G     = 26;
constexpr uint8_t PIN_RGBLED_B     = 27;

constexpr char *const TNGR_USERNAME    = "";
constexpr char *const TNGR_DEVICE_ID   = "";
constexpr char *const TNGR_DEVICE_CRED = "";

constexpr char *const WIFI_SSID        = "";
constexpr char *const WIFI_PASSWORD    = "";

constexpr uint8_t AUTO_BYPASS_WARNING_AMOUNT = 10;
constexpr uint8_t AUTO_BYPASS_FOOD_AMOUNT    = 20;

constexpr uint8_t COLIDX_RGB_VMAP[9][3] = {
    {0xFF, 0x00, 0x00},
    {0xFF, 0x1C, 0x00},
    {0xFF, 0x5E, 0x00},
    {0x2F, 0xFF, 0x00},
    {0x00, 0xFF, 0xFF},
    {0x00, 0x00, 0xFF},
    {0x80, 0x00, 0x80},
    {0xFF, 0x50, 0xAF}
};

enum class LEDState : uint8_t {
    OFF,
    ON,
    BLINK
};

enum class ColourIndex : uint8_t {
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

wl_status_t  wifi_status            = WL_IDLE_STATUS;

bool     is_running_automatically   = false;
bool     is_emergency_halt_on       = false;
bool     manual_feed_warning        = false;
bool     allow_neglect_bypass       = false;
uint16_t auto_feed_amount_g         = 0;
uint16_t manual_feed_amount_g       = 0;
uint16_t remaining_food_threshold_g = 0;

uint8_t  ignored_warning_amount     = 0;

bool     is_dispending              = false;
float    fed_food_remaining_g       = 0.0f;

void connect_to_wifi() noexcept {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

constexpr void set_rgb_led(
    const LEDState    state,
    const ColourIndex colour                 = ColourIndex::WHITE,
    const uint8_t     repeat_amount          = 0,
    const uint16_t    blink_up_duration_ms   = 0,
    const uint16_t    blink_down_duration_ms = 0
) noexcept {
    const uint8_t *COLOUR_RGB = COLIDX_RGB_VMAP[static_cast<uint8_t>(colour)];

    if (static_cast<uint8_t>(state) <= 1 || (static_cast<uint16_t>(repeat_amount) | blink_up_duration_ms | blink_down_duration_ms) == 0) {
        const bool useRGBValue = static_cast<bool>(state);

        analogWrite(PIN_RGBLED_R, useRGBValue ? COLOUR_RGB[0] : 0x00);
        analogWrite(PIN_RGBLED_G, useRGBValue ? COLOUR_RGB[1] : 0x00);
        analogWrite(PIN_RGBLED_B, useRGBValue ? COLOUR_RGB[2] : 0x00);

        return;
    }

    set_rgb_led(LEDState::OFF);
    delay(blink_down_duration_ms);

    for (uint8_t blinked = 0; blinked < repeat_amount; ++blinked) {
        set_rgb_led(LEDState::ON, colour);
        delay(blink_up_duration_ms);
        set_rgb_led(LEDState::OFF);
        delay(blink_down_duration_ms);
    }

    set_rgb_led(LEDState::OFF);
}

void update_device_property() noexcept {
    Thing.get_property("general_property", data);

    is_running_automatically   = static_cast<bool>    (data["is_running_automatically"]);
    is_emergency_halt_on       = static_cast<bool>    (data["is_emergency_halt_on"]);
    allow_neglect_bypass       = static_cast<bool>    (data["allow_neglect_bypass"]);
    auto_feed_amount_g         = static_cast<uint16_t>(data["auto_feed_amount_g"]);
    manual_feed_amount_g       = static_cast<uint16_t>(data["manual_feed_amount_g"]);
    manual_feed_warning        = static_cast<bool>    (data["manual_feed_warning"]);
    remaining_food_threshold_g = static_cast<uint16_t>(data["remaining_food_threshold_g"]);
}

void dump_property() noexcept {
    Serial.println("[!] NEW PROPERTY VALUES:");
    Serial.printf("    IS RUNNING AUTOMATICALLY: %s\n", is_running_automatically ? "YES" : "NO");
    Serial.printf("    IS EMERGENCY HALT ON    : %s\n", is_emergency_halt_on     ? "YES" : "NO");
    Serial.printf("    ALLOW BYPASS ON NEGLECT : %s\n", allow_neglect_bypass     ? "YES" : "NO");
    Serial.printf("    AUTO FEED AMOUNT (G)    : %u\n", auto_feed_amount_g);
    Serial.printf("    MANUAL FEED AMOUNT (G)  : %u\n", manual_feed_amount_g);
    Serial.printf("    MANUAL FEED WARNING     : %s\n", manual_feed_warning      ? "ENABLED" : "DISABLED");
    Serial.printf("    REMAINING THRESHOLD (G) : %u\n", remaining_food_threshold_g);
}

void dispend_food() noexcept {
    return;
}

void handle_tngr_update_signal(pson &in) {
    if (!static_cast<bool>(in))
        return;

    set_rgb_led(LEDState::OFF);
    delay(100);
    set_rgb_led(LEDState::ON, ColourIndex::CYAN);

    Serial.println("[!] NEW UPDATE SIGNAL RECEIVED");
    Serial.println("[/] GATHERING NEW PROPERTIES...");

    update_device_property();
    set_rgb_led(LEDState::BLINK, ColourIndex::CYAN, 3, 100, 100);

    dump_property();

    delay(500);
}

void handle_tngr_manual_feed_signal(pson &in) {
    if (!static_cast<bool>(in) || is_emergency_halt_on)
        return;

    set_rgb_led(LEDState::ON, ColourIndex::GREEN);

    dispend_food();

    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 3, 1000, 500);

    ignored_warning_amount = 0;
}

void setup() {
    Serial.begin(BOARD_BAUD);

    Serial.println("[/] SETTING UP PINS...");

    pinMode(PIN_RGBLED_R, OUTPUT);
    pinMode(PIN_RGBLED_G, OUTPUT);
    pinMode(PIN_RGBLED_B, OUTPUT);

    set_rgb_led(LEDState::ON, ColourIndex::RED);

    Serial.println("[/] INITIALISING PROGRAM...");
    delay(1000);

    set_rgb_led(LEDState::OFF);
    delay(100);

    set_rgb_led(LEDState::ON, ColourIndex::YELLOW);

    Serial.println("[/] ESTABLISHING WIFI CONNECTION...");

    while (WiFi.status() != WL_CONNECTED) {
        connect_to_wifi();

        set_rgb_led(LEDState::BLINK, ColourIndex::YELLOW, 3, 100, 100);
        delay(1000);
    }

    Serial.println("[+] WIFI CONNECTED!");

    set_rgb_led(LEDState::ON, ColourIndex::CYAN);

    Serial.println("[/] SETTING UP HANDLERS...");

    Thing["update_signal"] << handle_tngr_update_signal;
    Thing["manual_feed"]   << handle_tngr_manual_feed_signal;

    set_rgb_led(LEDState::OFF);

    Serial.println("[+] INITIALISATION COMPLETED!");

    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 3, 100, 100);

    set_rgb_led(LEDState::ON, ColourIndex::RED);
}

void loop() {

    wifi_status = WiFi.status();
    fed_food_remaining_g = 0.0f;

    // If WiFi is disconnected, keep retrying (every 1s) and do nothing else -
    // until the connection is re-established.
    if (wifi_status != WL_CONNECTED) {
        connect_to_wifi();

        set_rgb_led(LEDState::BLINK, ColourIndex::YELLOW, 3, 100, 100);
        delay(1000);

        return;
    }

    Thing.handle();

    // If these value stucks at its invalid state (all zeros)
    // Keep fetching the property until all the value is what it is meant to be.
    while (manual_feed_amount_g == 0 || auto_feed_amount_g == 0 || remaining_food_threshold_g == 0) {
        update_device_property();
        dump_property();

        set_rgb_led(LEDState::BLINK, ColourIndex::RED, 3, 100, 100);
    }

    // If emergency halt is on, do nothing, resetting the loop -
    // until emergency halt is turned off.
    if (is_emergency_halt_on) {
        set_rgb_led(LEDState::BLINK, ColourIndex::RED, 1, 1000, 500);

        return;
    }

    // If not automatic mode is disabled, do nothing even tho connections are met.
    if (!is_running_automatically) {
        set_rgb_led(LEDState::ON, ColourIndex::ORANGE);

        // Flash the light to warnn the user that the food should be refilled.
        // Only if it's enabled tho.
        if (fed_food_remaining_g < remaining_food_threshold_g) {
            ignored_warning_amount += 1;

            if (manual_feed_warning) {
                set_rgb_led(LEDState::BLINK, ColourIndex::ORANGE, 3, 100, 100);
                set_rgb_led(LEDState::ON, ColourIndex::ORANGE);
            }

            if (allow_neglect_bypass && (ignored_warning_amount > AUTO_BYPASS_WARNING_AMOUNT || AUTO_BYPASS_FOOD_AMOUNT)) {
                set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 3, 100, 100);
                set_rgb_led(LEDState::ON, ColourIndex::GREEN);

                dispend_food();

                set_rgb_led(LEDState::OFF);
            }

            delay(10000);
        }

        return;
    }

    if (fed_food_remaining_g < remaining_food_threshold_g) {
        set_rgb_led(LEDState::ON, ColourIndex::WHITE);

        dispend_food();

        return;
    }
}