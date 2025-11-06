/*
#! AUTOMATIC PET FEEDER [PROTOTYPE] - ESPino32 + Thinger.io
 __.
 .-".'                      .--.            _..._
 .' .'                     .'    \       .-""  __ ""-.
 /  /                     .'       : --..:__.-""  ""-. \
 :  :                     /         ;.d$$    sbp_.-""-:_:
 ;  :                    : ._       :P .-.   ,"TP
 :   \                    \  T--...-; : d$b  :d$b
 \   `.                   \  `..'    ; $ $  ;$ $
 `.   "-.                 ).        : T$P  :T$P
 \..---^..             /           `-'    `._`._
 .'        "-.       .-"                     T$$$b
 /             "-._.-"               ._        '^' ;
 :                                    \.`.         /
 ;                                -.   \`."-._.-'-'
 :                                 .'\   \ \ \ \
 ;  ;                             /:  \   \ \ . ;
 :   :                            ,  ;  `.  `.;  :
 ;    \        ;                     ;    "-._:  ;
 :      `.      :                     :         \/
 ;       /"-.    ;                    :
 :       /    "-. :                  : ;
 :     .'        T-;                 ; ;
 ;    :          ; ;                /  :
 ;    ;          : :              .'    ;
 :    :            ;:         _..-"\     :
 :     \           : ;       /      \     ;
 ;    . '.         '-;      /        ;    :
 ;  \  ; :           :     :         :    '-.
 '.._L.:-'           :     ;         ;    . `.
                     ;    :          :  \  ; :
                     :    '-..       '.._L.:-'
                     ;     , `.
                     :   \  ; :
                     '..__L.:-'
[WOOF WOOF] - I'm hungry now!
TODOs:
    - Complete servo's movement controller.
    - Complete tngr's calibration mechanism to calculate how long it has to open the servo.

*/

// [PREPROCESSORS]
// That's it.
#include <algorithm>
#include <Arduino.h>
#include <ThingerESP32.h>
#include <WiFi.h>

// [CONSTANTS]
// Here are some hard-coded values.
// There are infinitely more magic numbers to come.
constexpr uint32_t BOARD_BAUD = 115200;

constexpr uint8_t PIN_RELAY_L298N_5V   = 7;
constexpr uint8_t PIN_FSR_ESEN068_5V   = 20;
constexpr uint8_t PIN_FSR_ESEN068_DATA = 17;

constexpr uint8_t PIN_BUILT_IN_LED = 16;
constexpr uint8_t PIN_RGBLED_R     = 25;
constexpr uint8_t PIN_RGBLED_G     = 26;
constexpr uint8_t PIN_RGBLED_B     = 27;

constexpr char *const TNGR_USERNAME    = "YouAreNotGettingMyUsername";
constexpr char *const TNGR_DEVICE_ID   = "NorThis";
constexpr char *const TNGR_DEVICE_CRED = "ThieEither";

constexpr char *const WIFI_SSID        = "Ouilala";
constexpr char *const WIFI_PASSWORD    = "Baguette_Fetish67";

// These are here for secondary back-up plan if the owner forgot-
// to feed their pets.
// What an irresponsible asshole.
constexpr uint8_t AUTO_BYPASS_WARNING_AMOUNT = 10;
constexpr uint8_t AUTO_BYPASS_FOOD_AMOUNT    = 20;

// These are just {R, G, B} value map for each colour specified.
// Nothing fancy here.
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

// These are just functions' aliases.
// Their old names are disgusting to look at.
// All heil snake_case.
constexpr void     (*analog_write) (uint8_t, int)     = &analogWrite;
constexpr void     (*digital_write)(uint8_t, uint8_t) = &digitalWrite;
constexpr uint16_t (*analog_read)  (uint8_t)          = &analogRead;
constexpr int      (*digital_read) (uint8_t)          = &digitalRead;

// [ENUMS]
// All possible value of each.
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

enum class ServoState : int16_t {
    OFF = 0,
    ON  = 180
};

enum class DispendMode : uint8_t {
    SYSTEM_CALL,
    USER_CALL,
    CALIBRATION
};

// Oulala.
ThingerESP32 Thing(TNGR_USERNAME, TNGR_DEVICE_ID, TNGR_DEVICE_CRED);
pson         data;

// [STATE VARIABLES]
// These variables update all the time.
// Their default value for all the types except for boolean are invalid.
wl_status_t  wifi_status            = WL_IDLE_STATUS;

bool     is_running_automatically   = false;
bool     is_emergency_halt_on       = false;
bool     manual_feed_warning        = false;
bool     allow_neglect_bypass       = false;
uint16_t auto_feed_amount_g         = 0;
uint16_t manual_feed_amount_g       = 0;
uint16_t remaining_food_threshold_g = 0;
uint16_t calib_food_drop_in_100ms_g = 0;

uint8_t  ignored_warning_amount     = 0;

bool     is_dispending              = false;
float    current_food_remaining_g   = 0.0f;
int16_t  current_servo_angle_deg    = 0;

// [FUNCTIONS]
// Neither do I know what these are.
// I wrote these all by myself - but I can't explain a line of it.

// Function: update_device_property
// Description: Updates local variables with values from the device's general property data.
// Params: NONE
// Returns: NONE
void update_device_property() noexcept {
    // Get the latest property data from Thinger.io.
    Thing.get_property("general_property", data);

    // Update local variables with the received values.
    is_running_automatically   = static_cast<bool>    (data["is_running_automatically"]);
    is_emergency_halt_on       = static_cast<bool>    (data["is_emergency_halt_on"]);
    allow_neglect_bypass       = static_cast<bool>    (data["allow_neglect_bypass"]);
    auto_feed_amount_g         = static_cast<uint16_t>(data["auto_feed_amount_g"]);
    manual_feed_amount_g       = static_cast<uint16_t>(data["manual_feed_amount_g"]);
    manual_feed_warning        = static_cast<bool>    (data["manual_feed_warning"]);
    remaining_food_threshold_g = static_cast<uint16_t>(data["remaining_food_threshold_g"]);
    calib_food_drop_in_100ms_g = static_cast<uint16_t>(data["calibration_100ms_food_drop_g"]);
}

// Function: dump_property
// Description: Prints the current property values to the serial monitor.
// Params: NONE
// Returns: NONE
void dump_property() noexcept {
    Serial.println("[!] NEW PROPERTY VALUES:");
    Serial.printf("    IS RUNNING AUTOMATICALLY: %s\n", is_running_automatically ? "YES" : "NO");
    Serial.printf("    IS EMERGENCY HALT ON    : %s\n", is_emergency_halt_on     ? "YES" : "NO");
    Serial.printf("    ALLOW BYPASS ON NEGLECT : %s\n", allow_neglect_bypass     ? "YES" : "NO");
    Serial.printf("    AUTO FEED AMOUNT (G)    : %u\n", auto_feed_amount_g);
    Serial.printf("    MANUAL FEED AMOUNT (G)  : %u\n", manual_feed_amount_g);
    Serial.printf("    MANUAL FEED WARNING     : %s\n", manual_feed_warning      ? "ENABLED" : "DISABLED");
    Serial.printf("    REMAINING THRESHOLD (G) : %u\n", remaining_food_threshold_g);
    Serial.printf("    CALIBRATED DROP RATE (G): %u\n", calib_food_drop_in_100ms_g);
}

// Function: set_rgb_led
// Description: Sets or handles the state of the RGB LED.
// Params:
//     LEDState) state
//         The LED state (ON, OFF, etc.).
//     (ColourIndex) colour
//         Colour index for the RGB value map.
//     (uint8_t) repeat_amount
//         How many times the LED should blink.
//     (uint16_t) blink_up_duration_ms
//         How long the LED stays ON in each blink.
//     (uint16_t) blink_down_duration_ms
//         How long the LED stays OFF between blinks.
// Returns: NONE
constexpr void set_rgb_led(
    const LEDState    state,
    const ColourIndex colour                 = ColourIndex::WHITE,
    const uint8_t     repeat_amount          = 0,
    const uint16_t    blink_up_duration_ms   = 0,
    const uint16_t    blink_down_duration_ms = 0
) noexcept {
    // Get the RGB value for the given colour index.
    const uint8_t *COLOUR_RGB = COLIDX_RGB_VMAP[static_cast<uint8_t>(colour)];

    // If state is ON/OFF only, or blinking parameters are not set.
    if (static_cast<uint8_t>(state) <= 1 || (static_cast<uint16_t>(repeat_amount) | blink_up_duration_ms | blink_down_duration_ms) == 0) {
        // Convert LEDState to boolean.
        const bool use_rgb_value = static_cast<bool>(state);

        analog_write(PIN_RGBLED_R, use_rgb_value ? COLOUR_RGB[0] : 0x00);
        analog_write(PIN_RGBLED_G, use_rgb_value ? COLOUR_RGB[1] : 0x00);
        analog_write(PIN_RGBLED_B, use_rgb_value ? COLOUR_RGB[2] : 0x00);

        return;
    }

    // Turn off LED before starting blink sequence.
    set_rgb_led(LEDState::OFF);
    delay(blink_down_duration_ms);

    // Blink loop
    for (uint8_t blinked = 0; blinked < repeat_amount; ++blinked) {
        set_rgb_led(LEDState::ON, colour);
        delay(blink_up_duration_ms);
        set_rgb_led(LEDState::OFF);
        delay(blink_down_duration_ms);
    }

    // Leave LED off after blinking.
    set_rgb_led(LEDState::OFF);
}

// Function: get_current_fsr_g
// Description: Read the current force value from FSR (in g).
// Params: NONE
// Returns: (uint16_t) Read value from the FSR (in g).
uint16_t get_current_fsr_g() noexcept {
    // WAIT FOR THE FSR WIRE.
    // CANNOT CONTINUE DUE TO LACK OF HARDWARE.
    return 0;
}

// Function: calc_servo_movement_angle
// Description: Calculate how much and/or in which direction the servo has to rotate.
// Params:
//     (int16_t) angle_deg
//         Which direction does the servo want to face to.
// Returns: (uint16_t) How far the servo needs to rotate.
constexpr int16_t calc_servo_movement_angle(int16_t angle_deg) noexcept {
    // Round up the number to the range of 0-360 because that's all the circle has to offer.
    constexpr uint16_t FULL_CIRCLE_DEGREE = 360

    current_servo_angle_deg %= FULL_CIRCLE_DEGREE;
    angle_deg               %= FULL_CIRCLE_DEGREE;

    // This checl wether the servo needs to move clockwise or vise versa.
    return current_servo_angle_deg > angle_deg ? current_servo_angle_deg - angle_deg : current_servo_angle_deg + angle_deg;
}

// Function: set_servo_to_angle
// Description: Sets the rotation angle of the servo.
// Params:
//     (int16_t) angle_deg
//         Sets the servo to face to said direction.
// Returns: NONE
void set_servo_to_angle(const int16_t angle_deg) noexcept {
    // If the current value is the same as the new one - just ignore.
    if (current_servo_angle_deg == angle_deg)
        return;

    const int16_t angle_needed_to_rotate = calc_servo_movement_angle(angle_deg);

    // Will continue later.
    // analog_write(...);
}

// Function: set_servo_to_state_preset
// Description: Sets the rotation angle of the servo to a preset angle/degree based on the state.
// Params:
//     (ServoState) state
//         Sets the servo to face to the preset direction.
// Returns: NONE
void set_servo_to_state_preset(const ServoState state) noexcept {
    set_servo_to_angle(static_cast<int16_t>(state));
}

// Function: calc_dispend_time_g_ms
// Description: Calculates how long it takes (in ms) to dispend x (in g) amount of food.
// Params:
//     (uint16_t) food_amount_g
//         How much food (in g) is gonna be dispended.
// Returns: (uint32_t) How long it takes to dispend said amount of food (in ms).
constexpr uint16_t calc_dispend_time_g_ms(const uint16_t food_amount_g) noexcept {
    if (calib_food_drop_in_100ms_g == 0)
        return 0;

    // Wala!
    const float dispend_time_ms = (static_cast<float>(food_amount_g) / calib_food_drop_in_100ms_g) * 100.0f;
    return static_cast<uint16_t>(dispend_time_ms);
}

// Function: dispend_food
// Description: Handles the food dispensing process (servo control to be added later).
// Params:
//     (DispendMode) mode
//         How should this function behave.
// Returns: NONE
void dispend_food(const DispendMode mode) noexcept {
    set_rgb_led(LEDState::ON, ColourIndex::GREEN);

    const uint8_t u_mode = static_cast<uint8_t>(mode);

    // Calling with manual feed button & automatically called one is calculated differently.
    const uint16_t dispend_time_each_mode[3] = {
        /*SYSTEM_CALL*/ std::clamp(static_cast<uint16_t>(auto_feed_amount_g - current_food_remaining_g), static_cast<uint16_t>(0), auto_feed_amount_g),
        /*USER_CALL*/   calc_dispend_time_g_ms(manual_feed_amount_g),
        /*CALIBRATION*/ 100
    };

    if ((mode == DispendMode::SYSTEM_CALL && dispend_time_each_mode[u_mode] == 0) || (mode == DispendMode::USER_CALL && dispend_time_each_mode[u_mode] == 0))
        return;

    const uint16_t dispend_time_ms = calc_dispend_time_g_ms(dispend_time_each_mode[u_mode]);
    
    set_servo_to_state_preset(ServoState::ON);
    delay(dispend_time_ms);
    set_servo_to_state_preset(ServoState::OFF);

    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 3, 100, 100);
}

// [UTILITIES AND HANDLERS]

// Function: is_wifi_connected
// Description: Checks if the device is connected to WiFi.
// Params: NONE
// Returns: (bool) true if connected, false otherwise.
bool is_wifi_connected() noexcept {
    return WiFi.status() == WL_CONNECTED;
}

// Function: connect_to_wifi
// Descrption: Connects to a fucking WiFi.
// Params: NONE
// Returns: NONE
void connect_to_wifi() noexcept {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// Function: handle_wifi_disconnected
// Description: Tries to reconnect to WiFi while disconnected.
// Params: NONE
// Returns: NONE
void handle_wifi_disconnected() noexcept {
    while (!is_wifi_connected()) {
        set_rgb_led(LEDState::ON, ColourIndex::YELLOW);

        connect_to_wifi();

        // Blink to indicate reconnection attempt.
        set_rgb_led(LEDState::BLINK, ColourIndex::YELLOW, 3, 100, 100);
        delay(3000);
    }
}

// Function: is_data_valid
// Description: Checks if the data values are valid (non-zero).
// Params: NONE
// Returns: (bool) true if any data is invalid, false otherwise.
constexpr bool is_data_valid() noexcept {
    return manual_feed_amount_g == 0 || auto_feed_amount_g == 0 || remaining_food_threshold_g == 0 || calib_food_drop_in_100ms_g == 0;
}

// Function: handle_data_invalid
// Description: Repeatedly tries to update device properties until data becomes valid.
// Params: NONE
// Returns: NONE
void handle_data_invalid() noexcept {
    while (!is_data_valid()) {
        set_rgb_led(LEDState::ON, ColourIndex::PURPLE);

        update_device_property();

        // Blink to indicate update attempt.
        set_rgb_led(LEDState::BLINK, ColourIndex::PURPLE, 3, 100, 100);
        dump_property();
        delay(3000);
    }
}

// Function: is_met_dispend_condition
// Description: Checks if the remaining food meets the dispensing condition.
// Params: NONE
// Returns: (bool) true if remaining food is above or equal to the threshold, false otherwise.
constexpr bool is_met_dispend_condition() noexcept {
    return current_food_remaining_g >= remaining_food_threshold_g;
}

// Function: handle_automatic_mode_loop
// Description: Handles automatic feeding when the condition is met.
// Params: NONE
// Returns: NONE
void handle_automatic_mode_loop() noexcept {
    // Skip if condition not met.
    if (!is_met_dispend_condition())
        return;

    dispend_food(DispendMode::SYSTEM_CALL);
}

// Function: handle_manual_mode_loop
// Description: Handles manual feeding mode and warning logic.
// Params: NONE
// Returns: NONE
void handle_manual_mode_loop() noexcept {
    // Skip if condition not met.
    if (!is_met_dispend_condition())
        return;

    ++ignored_warning_amount;

    // Blink orange if manual feed warning is enabled.
    if (manual_feed_warning)
        set_rgb_led(LEDState::BLINK, ColourIndex::ORANGE, 3, 100, 100);

    // Auto bypass feed if ignored too many warnings and bypass is allowed.
    if (current_food_remaining_g < AUTO_BYPASS_FOOD_AMOUNT && ignored_warning_amount >= AUTO_BYPASS_WARNING_AMOUNT && allow_neglect_bypass) {
        dispend_food(DispendMode::SYSTEM_CALL);
        ignored_warning_amount = 0;
    }
}

// Function: handle_emergency_halt
// Description: Handles emergency halt state with a red blinking LED.
// Params: NONE
// Returns: NONE
void handle_emergency_halt() noexcept {
    set_rgb_led(LEDState::BLINK, ColourIndex::RED, 3, 1000, 500);
}

// Function: set_mode_corresponding_rgb_led_colour
// Description: Sets the RGB LED colour based on the current mode.
// Params: NONE
// Returns: NONE
void set_mode_corresponding_rgb_led_colour() noexcept {
    set_rgb_led(LEDState::ON, is_running_automatically ? ColourIndex::WHITE : ColourIndex::ORANGE);
}

// Function: handle_tngr_update_signal
// Description: Handles the update signal from Thinger.io and refreshes device properties.
// Params:
//     (pson&) in
//         Input payload from Thinger.io containing the update signal.
// Returns: NONE
void handle_tngr_update_signal(pson &in) noexcept {
    // If the payload is false (the button is released) - just ignore.
    if (!static_cast<bool>(in))
        return;

    // Blink for signal
    set_rgb_led(LEDState::OFF);
    delay(100);
    set_rgb_led(LEDState::ON, ColourIndex::CYAN);

    Serial.println("[!] NEW UPDATE SIGNAL RECEIVED");
    Serial.println("[/] GATHERING NEW PROPERTIES...");

    // Refresh/update the device property, then blink for signal again.
    update_device_property();
    set_rgb_led(LEDState::BLINK, ColourIndex::CYAN, 3, 100, 100);

    // Print all the current, new value onto the serial monitor.
    dump_property();

    delay(500);
}

// Function: handle_tngr_manual_feed_signal
// Description: Handles the manual feed signal from Thinger.io and triggers food dispensing.
// Params:
//     (pson&) in
//         Input payload from Thinger.io containing the manual feed signal.
// Returns: NONE
void handle_tngr_manual_feed_signal(pson &in) noexcept {
    // Ignore if button is released or emergency halt is active.
    if (!static_cast<bool>(in) || is_emergency_halt_on)
        return;

    // Nom nom nom nom.
    dispend_food(DispendMode::USER_CALL);

    // Reset ignored warning counter.
    ignored_warning_amount = 0;
}

// Function: handle_tngr_re_calibrate_signal
// Description: Handles the re-calibration signal from Thinger.io and update calibration property.
// Params:
//     (pson&) in
//         Input payload from Thinger.io containing the manual feed signal.
// Returns: NONE
void handle_tngr_re_calibrate_signal(pson &in) noexcept {
    // Ignore if button is released or emergency halt is active.
    if (!static_cast<bool>(in) || is_emergency_halt_on)
        return;

    set_rgb_led(LEDState::ON, ColourIndex::PURPLE);
    delay(300);

    // const float avg_weight_pre_dispense = ... / 3.0f;
    // READ VALUE 3 TIMES THEN AVERAGE 0-> STORE
    // dispend_food(100);
    // DISPEND FOOD FOR 100MS
    // READ VALUE -> COMPARE TO THE FIRST ONE, STORE.

    // Thing.write_property("calibration_100ms_food_drop_g", ...);

    set_rgb_led(LEDState::BLINK, ColourIndex::PURPLE, 3, 100, 100);
}

// Function: setup_pin_mode
// Description: Sets all required pin modes for the RGB LED.
// Params: NONE
// Returns: NONE
void setup_pin_mode() noexcept {
    pinMode(PIN_RGBLED_R, OUTPUT);
    pinMode(PIN_RGBLED_G, OUTPUT);
    pinMode(PIN_RGBLED_B, OUTPUT);

    pinMode(PIN_RELAY_L298N_5V, OUTPUT);
    pinMode(PIN_FSR_ESEN068_5V, OUTPUT);
    pinMode(PIN_FSR_ESEN068_DATA, INPUT);
}

// Function: setup_handlers
// Description: Registers all Thinger.io property handlers.
// Params: NONE
// Returns: NONE
void setup_handlers() noexcept {
    Thing["update_signal"] << handle_tngr_update_signal;
    Thing["manual_feed"]   << handle_tngr_manual_feed_signal;
    Thing["re_calibrate_signal"] << handle_tngr_re_calibrate_signal;
}

// [MAIN AREA - KEEP OUT!!]
// Have a bloody nice day!
void setup() {
    Serial.begin(BOARD_BAUD);

    Serial.println("[/] SETTING UP PINS...");
    setup_pin_mode();

    Serial.println("[/] INITIALISING PROGRAM...");

    // Quick startup LED sequence.
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

    // Blink to indicate success, then idle LED.
    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 3, 100, 100);
    set_rgb_led(LEDState::ON, ColourIndex::RED);
}

void loop() {
    // Reconnect if Wi-Fi drops.
    if (!is_wifi_connected()) {
        handle_wifi_disconnected();
        return;
    }

    // Keep Thinger.io connection alive.
    Thing.handle();

    // Ensure required data is valid before continuing.
    if (!is_data_valid()) {
        handle_data_invalid();
        return;
    }

    // Stop all operations if emergency halt is active.
    if (is_emergency_halt_on) {
        handle_emergency_halt();
        return;
    }

    // Set LED according to current mode.
    set_mode_corresponding_rgb_led_colour();

    // Run active mode loop.
    if (is_running_automatically)
        handle_automatic_mode_loop();
    else
        handle_manual_mode_loop();

    // Maintain mode LED state.
    set_mode_corresponding_rgb_led_colour();
}