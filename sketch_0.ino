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
float    current_food_remaining_g       = 0.0f;

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
        const bool use_rgb_value = static_cast<bool>(state);

        analogWrite(PIN_RGBLED_R, use_rgb_value ? COLOUR_RGB[0] : 0x00);
        analogWrite(PIN_RGBLED_G, use_rgb_value ? COLOUR_RGB[1] : 0x00);
        analogWrite(PIN_RGBLED_B, use_rgb_value ? COLOUR_RGB[2] : 0x00);

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
    set_rgb_led(LEDState::ON, ColourIndex::GREEN);

    // DISPEND FOOD CODE LATER - I HAVE NO SERVO YET.

    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 3, 100, 100);
}

void handle_tngr_update_signal(pson &in) noexcept {
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

void handle_tngr_manual_feed_signal(pson &in) noexcept {
    if (!static_cast<bool>(in) || is_emergency_halt_on)
        return;

    dispend_food();

    ignored_warning_amount = 0;
}

bool is_wifi_connected() noexcept {
    return WiFi.status() == WL_CONNECTED;
}

bool is_data_valid() noexcept {
    return manual_feed_amount_g == 0 || auto_feed_amount_g == 0 || remaining_food_threshold_g == 0;
}

bool is_met_dispend_condition() noexcept {
    return current_food_remaining_g >= remaining_food_threshold_g;
}

void handle_wifi_disconnected() noexcept {
    while (!is_wifi_connected()) {
        set_rgb_led(LEDState::ON, ColourIndex::YELLOW);

        connect_to_wifi();

        set_rgb_led(LEDState::BLINK, ColourIndex::YELLOW, 3, 100, 100);
        delay(3000);
    }
}

void handle_data_invalid() noexcept {
    while (!is_data_valid()) {
        set_rgb_led(LEDState::ON, ColourIndex::PURPLE);

        update_device_property();

        set_rgb_led(LEDState::BLINK, ColourIndex::PURPLE, 3, 100, 100);
        dump_property();
        delay(3000);
    }
}

void handle_emergency_halt() noexcept {
    set_rgb_led(LEDState::BLINK, ColourIndex::RED, 3, 1000, 500);
}

void handle_automatic_mode_loop() noexcept {
    if (!is_met_dispend_condition())
        return;

    dispend_food();
}

void handle_manual_mode_loop() noexcept {
    if (!is_met_dispend_condition())
        return;

    ++ignored_warning_amount;

    if (manual_feed_warning)
        set_rgb_led(LEDState::BLINK, ColourIndex::ORANGE, 3, 100, 100);

    if (current_food_remaining_g < AUTO_BYPASS_FOOD_AMOUNT && ignored_warning_amount >= AUTO_BYPASS_WARNING_AMOUNT && allow_neglect_bypass) {
        dispend_food();
        ignored_warning_amount = 0;
    }
}

void set_mode_corresponding_rgb_led_colour() noexcept {
    set_rgb_led(LEDState::ON, is_running_automatically ? ColourIndex::WHITE : ColourIndex::ORANGE);
}

void setup_pin_mode() noexcept {
    pinMode(PIN_RGBLED_R, OUTPUT);
    pinMode(PIN_RGBLED_G, OUTPUT);
    pinMode(PIN_RGBLED_B, OUTPUT);
}

void setup_handlers() noexcept {
    Thing["update_signal"] << handle_tngr_update_signal;
    Thing["manual_feed"]   << handle_tngr_manual_feed_signal;
}

void setup() {
    Serial.begin(BOARD_BAUD);

    Serial.println("[/] SETTING UP PINS...");

    setup_pin_mode();

    Serial.println("[/] INITIALISING PROGRAM...");

    set_rgb_led(LEDState::ON, ColourIndex::RED);
    delay(1000);
    set_rgb_led(LEDState::OFF);
    delay(100);

    Serial.println("[/] ESTABLISHING WIFI CONNECTION...");

    if (!is_wifi_connected()) {
        handle_wifi_disconnected();
    }

    Serial.println("[+] WIFI CONNECTED!");
    Serial.println("[/] SETTING UP HANDLERS...");

    setup_handlers();

    Serial.println("[+] INITIALISATION COMPLETED!");

    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 3, 100, 100);
    set_rgb_led(LEDState::ON, ColourIndex::RED);
}

void loop() {
    if (!is_wifi_connected()) {
        handle_wifi_disconnected();

        return;
    }

    Thing.handle();

    if (!is_data_valid()) {
        handle_data_invalid();

        return;
    }

    if (is_emergency_halt_on) {
        handle_emergency_halt();

        return;
    }

    set_mode_corresponding_rgb_led_colour();

    if (is_running_automatically) {
        handle_automatic_mode_loop();
    }
    else {
        handle_manual_mode_loop();
    }

    set_mode_corresponding_rgb_led_colour();
}