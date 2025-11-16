/*
#! AUTOMATIC PET FEEDER [PROTOTYPE] - ESPino32 + Thinger.io
    .
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
*/

// [PREPROCESSORS]

// That's it.
#include <algorithm>
#include <cmath>

#include <Arduino.h>
#include <ThingerESP32.h>
#include <WiFi.h>
#include <HX711_ADC.h>
#include <L298N.h>

// [ALIASES]

// I hate how it looks, so I changed it.

// Their old names are disgusting to look at.
// All heil snake_case.
#define pin_mode pinMode

// Standard a/d w/r functions:
#define analog_write  analogWrite
#define analog_read   analogRead
#define digital_write digitalWrite
#define digital_read  digitalRead

// HX711_ADC's:
#define set_calibration_factor setCalFactor
#define get_data               getData

// L298N's:
#define set_speed setSpeed

// STDs':
#define get_timestamp_ms millis

// [CONSTANTS]

// Here are some hard-coded values.
// There are infinitely more magic numbers to come.

// Physical board constants:
constexpr uint32_t BOARD_BAUD = 115200;

/*
[BLUEPRINT]

#[ESPino32]
 [GPIO 4] ------------------------------------------------------------+
              #[L298N RELAY]    #[10K POTENTIOMETER]   #[9V BATTERY]  |
               [12V] <---------- [PIN LEFT]       +---- [ANODE]       |
               [GND] <---+       [PIN MIDDLE] <---+     [CATHODE] <-+ |
 [GPIO 18] --> [ENA]     |                                          | |
 [GPIO 21] --> [IN 1]    |      #[DC MOTOR]                         | |
 [GPIO 22] --> [IN 2]    | +---> [ANODE]                            | |
               [OUT 1] --=-+     [CATHODE] -------------------------+ |
  NOT USED <-- [OUT 2]   |                                            |
  *I couldn't move the-  |      #[HX711 <- LOAD CELL]                 |
  damn screw >:(         |       [VCC] <------------------------------+
                         | +---- [DATA]
             #[RGB LED]  | | +-> [SCK]
 [GPIO 25] -> [R]        | | |   [GND] <-+
 [GPIO 26] -> [G]        | | |           |
 [GPIO 27] -> [B]        | | |           |
              [GND] <----+-=-=-+---------+
 [GPIO 32] <---------------+ | |
 [GPIO 33] ------------------+ |
 [GND] ------------------------+
*/

// LEDs.
constexpr uint8_t PIN_BUILT_IN_LED = 16;
constexpr uint8_t PIN_RGBLED_R     = 25;
constexpr uint8_t PIN_RGBLED_G     = 26;
constexpr uint8_t PIN_RGBLED_B     = 27;

// Bullshits.
constexpr uint8_t PIN_RELAY_L298N_ENA = 18;
constexpr uint8_t PIN_RELAY_L298N_1   = 21;
constexpr uint8_t PIN_RELAY_L298N_2   = 22;

// More bullshits.
constexpr uint8_t PIN_LOAD_CELL_5V   = 4;
constexpr uint8_t PIN_LOAD_CELL_DATA = 32;
constexpr uint8_t PIN_LOAD_CELL_SCK  = 33;

// Thinger's constants:
constexpr char *const TNGR_USERNAME    = "";
constexpr char *const TNGR_DEVICE_ID   = "";
constexpr char *const TNGR_DEVICE_CRED = "";

// WiFi's cconstants:
constexpr char *const WIFI_SSID     = "";
constexpr char *const WIFI_PASSWORD = "";

// These are here for secondary back-up plan if the owner forgot-
// to feed their pets.
// What an irresponsible arsehole.
constexpr uint8_t AUTO_BYPASS_WARNING_AMOUNT = 10;
constexpr uint8_t AUTO_BYPASS_FOOD_AMOUNT    = 20;

// Accuracy related:
constexpr uint16_t LOAD_CELL_STABILISE_TIME_MS  = 5000;
constexpr float    LOAD_CELL_CALIBRATION_FACTOR = -1100.0f;
constexpr int8_t   LOAD_CELL_SAMPLE_AMOUNT_POW  = 7;
constexpr uint16_t LOAD_CELL_GHOST_WEIGHT_G     = 1201;

// [ENUMS]

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

enum class DispenseMode : uint8_t {
    AUTOMATIC,
    MANUAL
};

// [VALUE MAPS]

// ColourIndex enum -> RGB.
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

// [CONSTRUCTOR INITIALISATION]

// Oulala.
HX711_ADC LoadCell(PIN_LOAD_CELL_DATA, PIN_LOAD_CELL_SCK);

// Bloody hell.
ThingerESP32 Thing(TNGR_USERNAME, TNGR_DEVICE_ID, TNGR_DEVICE_CRED);
pson         data;

// Vroom vrrom mewo.
L298N Motor(PIN_RELAY_L298N_ENA, PIN_RELAY_L298N_1, PIN_RELAY_L298N_2);

// [STATE VARIABLES]
// These variables update all the time.
// Their default value for all the types except for boolean are invalid.

// Connection variable:
wl_status_t  wifi_status            = WL_IDLE_STATUS;

// Device property variables:
bool     is_running_automatically   = false;
bool     is_emergency_halt_on       = false;
bool     manual_feed_warning        = false;
bool     allow_neglect_bypass       = false;
uint16_t auto_feed_amount_g         = 0;
uint16_t manual_feed_amount_g       = 0;
uint16_t remaining_food_threshold_g = 0;
uint8_t  motor_rotation_speed       = 0;

// Current device's physically readable/trackable variables:
bool     is_dispensing              = false;
bool     is_taring                  = false;
uint8_t  ignored_warning_amount     = 0;
uint16_t tare_amount_since_boot     = 0;

// Time keepers:
uint64_t timestamp_program_initialised_ms  = 0;
uint64_t timestamp_current_program_loop_ms = 0;
uint64_t time_since_last_tngr_update_ms    = 0;

// [FUNCTION FORWARD DECLRATIONS]
// Some of these guys don't know that themselves exist.

// Low-level hardware / setup:
void     set_rgb_led(
             const LEDState,
             const ColourIndex,
             const uint8_t,
             const uint16_t,
             const uint16_t
         )                  noexcept;
void     update_load_cell() noexcept;
uint16_t get_load_cell_g()  noexcept;

// Device state checks:
bool is_wifi_connected()              noexcept;
bool is_data_valid()                  noexcept;
bool is_met_dispense_condition()      noexcept;
bool is_device_running_for_too_long() noexcept;

// Delay / timing helpers:
void delay_with_update(uint32_t) noexcept;

// WiFi / connectivity:
void connect_to_wifi()          noexcept;
void handle_wifi_disconnected() noexcept;
void handle_data_invalid()      noexcept;

// Core tngr logic / updates:
void listen_for_tngr_interaction() noexcept;
void distribute_changes()          noexcept;
void update_device_property()      noexcept;
void dump_property()               noexcept;

// void handle_tngr_update_signal()      noexcept; \  
// void handle_tngr_manual_feed_signal() noexcept;  } Cannot forward declare these guys because the compiler hated it.
// void handle_tngr_tare_signal()        noexcept; /  
void handle_never_tared()             noexcept;
void finalise_handler()               noexcept;

// Operation loops / dispense:
void dispense_food(const DispenseMode) noexcept;

void handle_automatic_mode_loop()         noexcept;
void handle_manual_mode_loop()            noexcept;
void handle_device_running_for_too_long() noexcept;
void handle_emergency_halt()              noexcept;

// Setup helpers:
void setup_pin_mode()     noexcept;
void setup_load_cell()    noexcept;
void setup_tngr_handler() noexcept;

// [FUNCTIONS]
// Neither do I know what these are.
// I wrote these all by myself - but I can't explain a line of it.

// [FUNCTION SECTION - HARDWARE CONTROLLERS]

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
//         How long the LED stays ON between blinks.
//     (uint16_t) blink_down_duration_ms
//         How long the LED stays OFF between blinks.
// Returns: NONE
void set_rgb_led(
    const LEDState    state,
    const ColourIndex colour                 = ColourIndex::WHITE,
    const uint8_t     repeat_amount          = 0,
    const uint16_t    blink_up_duration_ms   = 0,
    const uint16_t    blink_down_duration_ms = 0
) noexcept {
    // Get the RGB value for the given colour index.
    const uint8_t *COLOUR_RGB = COLIDX_RGB_VMAP[static_cast<uint8_t>(colour)];

    // Serial.printf("RGB Signal: %s (%d)\n", state == LEDState::OFF ? "OFF" : state == LEDState::ON ? "ON" : "BLINK", state == LEDState::OFF ? -1 : static_cast<uint8_t>(colour));

    // If state is ON/OFF only, or blinking parameters are not set.
    if (static_cast<uint8_t>(state) <= 1 || (static_cast<uint16_t>(repeat_amount) | blink_up_duration_ms | blink_down_duration_ms) == static_cast<uint16_t>(0)) {
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

// Function: update_load_cell
// Description: Don't question me, please.
// Params: NONE
// Returns: NONE
void update_load_cell() noexcept {
    for (uint8_t _ = 0; _ < 3; ++_)
        LoadCell.update();
}

// Function: get_load_cell_g
// Description: Read load cell value.
// Params: NONE
// Returns: (uint16_t) current read weight on the load cell.
uint16_t get_load_cell_g() noexcept {
    uint32_t sum_of_weight_g = 0;

    // Get an average reading value because the load cell is constantly fluctuating.
    for (uint8_t read = 0; read < (1 << LOAD_CELL_SAMPLE_AMOUNT_POW); ++read) {
        update_load_cell();

        float raw = LoadCell.get_data();

        // WEIGHT MUST NOT BE NEGATIVE BECAUSE OTHERWISE THE FOOD WOULD FLY OFF.
        sum_of_weight_g += raw < 0 ? 0 : raw;
    }

    // Divided by the amount of reading amount.
    const uint16_t avg_weight_g = sum_of_weight_g >> LOAD_CELL_SAMPLE_AMOUNT_POW;

    return avg_weight_g < 0 ? 0 : avg_weight_g;
}

// [FUNCTION SECTION - DEVICE STATE CHECKERS]

// Function: is_met_dispense_condition
// Description: Checks if the remaining food meets the dispensing condition.
// Params: NONE
// Returns: (bool) true if remaining food is above or equal to the threshold, false otherwise.
bool is_met_dispense_condition() noexcept {
    return get_load_cell_g() < remaining_food_threshold_g;
}

// Function: is_device_running_for_too_long
// Description: Checks if the device is running for too long.
// Params: NONE
// Returns: (bool) true if the device is indeed running for too long, false otherwise.
bool is_device_running_for_too_long() noexcept {
    // 30 days uptime limit in ms;
    constexpr uint32_t UPTIME_LIMIT_MS = 30UL * 24UL * 60UL * 60UL * 1000UL;

    return get_timestamp_ms() > UPTIME_LIMIT_MS;
}

// [FUNCTION SECTION - TIMING HELPERS]

// Function: delay_with_update
// Description: Replaces the built-in delay function because it makes my thinger.io dashbaord feels unresponsive.
// Params: NONE
// Returns: NONE
void delay_with_update(const uint32_t delay_ms) noexcept {
    const uint32_t DELAY_END_MS = get_timestamp_ms() + delay_ms;

    // Keeps delaying bit-by-bit until time is reached.
    while (get_timestamp_ms() < DELAY_END_MS) {
        listen_for_tngr_interaction();
        delay(1);

        if (is_emergency_halt_on)
            handle_emergency_halt();
    }
}

// [FUNCTION SECTION - WIFI / NETWORK CONNECTIVITY]

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
        delay(3000);

        // Blink to indicate reconnection attempt.
        set_rgb_led(LEDState::BLINK, ColourIndex::YELLOW, 2, 100, 100);
    }
}

// [FUNCTION SECTION - DATA VALIDITY]

// Function: is_data_valid
// Description: Checks if the data values are valid (non-zero).
// Params: NONE
// Returns: (bool) true if any data is valid, false otherwise.
bool is_data_valid() noexcept {
    return manual_feed_amount_g != 0 && auto_feed_amount_g != 0 && remaining_food_threshold_g != 0;
}

// Function: handle_data_invalid
// Description: Repeatedly tries to update device properties until data becomes valid.
// Params: NONE
// Returns: NONE
void handle_data_invalid() noexcept {
    while (!is_data_valid()) {
        set_rgb_led(LEDState::ON, ColourIndex::CYAN);

        update_device_property();
        delay(3000);

        // Blink to indicate update attempt.
        set_rgb_led(LEDState::BLINK, ColourIndex::CYAN, 2, 100, 100);
        dump_property();
    }
}

// [FUNCTION SECTION - CORE THINGER LOGIC]

// Function: listen_for_tngr_interaction
// Description: Listens for signals come from Thinger.io dashboard.
// Params: NONE
// Returns: NONE
void listen_for_tngr_interaction() noexcept {
    const uint64_t CURRENT_TIME_MS = get_timestamp_ms();
    
    // Only call handle function if its 500ms passed.
    if (CURRENT_TIME_MS - time_since_last_tngr_update_ms > 250 || time_since_last_tngr_update_ms == 0) {
        Serial.println("[!] LISTENING TO THINGER.IO");
        Thing.handle();

        time_since_last_tngr_update_ms = CURRENT_TIME_MS;
        return;
    }
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

    // temporarily update the variable.
    is_emergency_halt_on = false;

    Motor.stop();

    // Blink for signal
    set_rgb_led(LEDState::OFF);
    delay(100);
    set_rgb_led(LEDState::ON, ColourIndex::CYAN);

    Serial.println("[!] NEW UPDATE SIGNAL RECEIVED");
    Serial.println("[/] GATHERING NEW PROPERTIES...");

    // Refresh/update the device property, then blink for signal again.
    update_device_property();
    set_rgb_led(LEDState::BLINK, ColourIndex::CYAN, 2, 100, 100);

    // Print all the current, new value onto the serial monitor.
    dump_property();

    delay(500);

    finalise_handler();
}

// Function: handle_tngr_manual_feed_signal
// Description: Handles the manual feed signal from Thinger.io and triggers food dispensing.
// Params:
//     (pson&) in
//         Input payload from Thinger.io containing the manual feed signal.
// Returns: NONE
void handle_tngr_manual_feed_signal(pson &in) noexcept {
    // Ignore if button is released or emergency halt is active.
    if (!static_cast<bool>(in) || is_emergency_halt_on || is_dispensing)
        return;

    // Nom nom nom nom.
    dispense_food(DispenseMode::MANUAL);

    // Reset ignored warning counter.
    ignored_warning_amount = 0;

    finalise_handler();
}

// Function: handle_tngr_tare_signal
// Description: Handles the tare signal from Thinger.io and update the load-cell property.
// Params:
//     (pson&) in
//         Input payload from Thinger.io containing the manual feed signal.
// Returns: NONE
void handle_tngr_tare_signal(pson &in) noexcept {
    // Ignore if button is released or emergency halt is active.
    if (!static_cast<bool>(in) || is_emergency_halt_on || is_taring)
        return;

    is_taring = true;
    Motor.stop();
    Serial.println("[!] TARE SIGNAL RECEIVED");
    Serial.println("[!] PLEASE REMOVE ALL WEIGHT AND LEAVE AN EMPTY PLATE ON THE LOAD CELL.");

    // Warns user to clear everything off and place blank plate/dish on the load cell.
    set_rgb_led(LEDState::ON, ColourIndex::RED);
    delay(10000);
    set_rgb_led(LEDState::BLINK, ColourIndex::RED, 2, 100, 100);

    Serial.println("[/] CALIBRATING...");
    Serial.println("[!] PLEASE DO NOT TOUCH/MOVE ANYTHING ON THE LOAD CELL.");

    set_rgb_led(LEDState::ON, ColourIndex::YELLOW);

    // Tare until the load cell reading is 0.
    while (true) {
        update_load_cell();
        LoadCell.tare();

        delay(3000);

        if (get_load_cell_g() == 0)
            break;
    }

    tare_amount_since_boot += 1;
    set_rgb_led(LEDState::BLINK, ColourIndex::YELLOW, 2, 100, 100);

    set_rgb_led(LEDState::ON, ColourIndex::GREEN);
    delay(1000);

    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 2, 100, 100);

    finalise_handler();

    is_taring = false;
    Serial.println("[!] TARE COMPLETED!");
}

// Function: distribute_changes
// Description: Apply changes to device configuration after update.
// Params: NONE
// Returns: NONE
void distribute_changes() noexcept {
    // IDK what to do. have a nice day.
    Motor.set_speed(motor_rotation_speed);
}

// Function: finalise_handler
// Description: Clear up things/finalise things up before exitting handler's scope.
// Params: NONE
// Returns: NONE
void finalise_handler() noexcept {
    // :D
    set_rgb_led(LEDState::ON, is_running_automatically ? ColourIndex::WHITE : ColourIndex::ORANGE);
}

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
    motor_rotation_speed       = static_cast<uint8_t> (data["motor_rotation_speed"]);

    distribute_changes();
}

// Function: dump_property
// Description: Prin`ts the current property values to the serial monitor.
// Params: NONE
// Returns: NONE
void dump_property() noexcept {
    Serial.println("[!] NEW PROPERTY VALUES:");
    Serial.printf("    IS RUNNING AUTOMATICALLY: %s\n", is_running_automatically ? "YES" : "NO");
    Serial.printf("    IS EMERGENCY HALT ON    : %s\n", is_emergency_halt_on     ? "YES" : "NO");
    Serial.printf("    ALLOW BYPASS ON NEGLECT : %s\n", allow_neglect_bypass     ? "YES" : "NO");
    Serial.printf("    AUTO FEED AMOUNT(G)     : %u\n", auto_feed_amount_g);
    Serial.printf("    MANUAL FEED AMOUNT(G)   : %u\n", manual_feed_amount_g);
    Serial.printf("    MANUAL FEED WARNING     : %s\n", manual_feed_warning      ? "ENABLED" : "DISABLED");
    Serial.printf("    REMAINING THRESHOLD(G)  : %u\n", remaining_food_threshold_g);
    Serial.printf("    MOTOR ROTATION SPEED    : %d\n", motor_rotation_speed);
}

// [FUNCTION SECTION - OPERATION LOOPS / DISPENSE]

// Function: dispense_food
// Description: Handles the food dispensing process (servo control to be added later).
// Params:
//     (DispenseMode) mode
//         How should this function behave.
// Returns: NONE
void dispense_food(const DispenseMode mode) noexcept {
    if (is_emergency_halt_on)
        return;

    set_rgb_led(LEDState::ON, ColourIndex::GREEN);
    Serial.printf("Current weight: %d\n", get_load_cell_g());

    const bool initial_is_running_automatically = is_running_automatically;
    const uint16_t before_weight_g = get_load_cell_g();
    const uint16_t target_choices[2] = {
        /*AUTOMATIC*/ std::clamp(auto_feed_amount_g - before_weight_g, 0, static_cast<int>(auto_feed_amount_g)),
        /*MANUAL*/    before_weight_g + manual_feed_amount_g
    };

    const uint16_t target_weight_g = target_choices[static_cast<uint8_t>(mode)];

    if (mode == DispenseMode::AUTOMATIC && target_weight_g == 0 || get_load_cell_g() >= target_weight_g)
        return;

    // Keep dispensing until I love you.
    while (get_load_cell_g() < target_weight_g) {
        Serial.printf("Current weight: %d\n", get_load_cell_g());
        set_rgb_led(LEDState::ON, ColourIndex::GREEN);

        // How could I forgot this...
        if (is_emergency_halt_on) {
            is_dispensing = false;
            handle_emergency_halt();
        }

        is_dispensing = true;
        Motor.forward();

        listen_for_tngr_interaction();

        // If the mode is changed, exit immediately.
        if (initial_is_running_automatically != is_running_automatically) {
            Motor.stop();
            is_dispensing = false;

            return;
        }
    }

    Motor.stop();
    is_dispensing = false;

    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 2, 100, 100);
}

// Function: handle_automatic_mode_loop
// Description: Handles automatic feeding when the condition is met.
// Params: NONE
// Returns: NONE
void handle_automatic_mode_loop() noexcept {
    // Skip if condition not met.
    if (!is_met_dispense_condition())
        return;

    dispense_food(DispenseMode::AUTOMATIC);

    finalise_handler();
}

// Function: handle_manual_mode_loop
// Description: Handles manual feeding mode and warning logic.
// Params: NONE
// Returns: NONE
void handle_manual_mode_loop() noexcept {
    // Skip if condition not met.
    if (!is_met_dispense_condition()) {
        ignored_warning_amount = 0;
        listen_for_tngr_interaction();
        return;
    }

    ++ignored_warning_amount;
    listen_for_tngr_interaction();

    if (is_emergency_halt_on)
        handle_emergency_halt();

    // Blink orange if manual feed warning is enabled.
    if (manual_feed_warning) {
        set_rgb_led(LEDState::BLINK, ColourIndex::ORANGE, 2, 100, 100);
        set_rgb_led(LEDState::ON, ColourIndex::ORANGE);

        listen_for_tngr_interaction();
        delay_with_update(5000);
    }

    // Auto bypass feed if ignored too many warnings and bypass is allowed.
    if (get_load_cell_g() < AUTO_BYPASS_FOOD_AMOUNT && ignored_warning_amount >= AUTO_BYPASS_WARNING_AMOUNT && allow_neglect_bypass) {
        dispense_food(DispenseMode::AUTOMATIC);
        ignored_warning_amount = 0;
    }

    finalise_handler();
}

// Function: handle_device_running_for_too_long
// Description: Prevents get_timestamp_ms() to go nuts because of integer overflow.
// Params: NONE
// Returns: NONE
void handle_device_running_for_too_long() noexcept {
    set_rgb_led(LEDState::ON, ColourIndex::RED);
    delay_with_update(10000);

    // Restart the board.
    ESP.restart();
}

// Function: handle_never_tared
// Description: Pause everything on this colour until it's tared.
// Params: NONE
// Returns: NONE
void handle_never_tared() noexcept {

    // If never tare before, require first initial tare. (prevent ghost weight.)
    while (tare_amount_since_boot == 0) {
        set_rgb_led(LEDState::ON, ColourIndex::PURPLE);

        Serial.printf("Weight: %d\n", get_load_cell_g());

        // Do nothing until tared.
        listen_for_tngr_interaction();
    }

    finalise_handler();
}

// Function: handle_emergency_halt
// Description: Handles emergency halt state with a red blinking LED.
// Params: NONE
// Returns: NONE
void handle_emergency_halt() noexcept {
    // Making sure the motor is halted.
    if (!is_emergency_halt_on)
        return;

    Motor.stop();

    uint64_t initial_time = get_timestamp_ms();

    while (is_emergency_halt_on) {
        uint64_t current_time = get_timestamp_ms();

        listen_for_tngr_interaction();

        if (!is_emergency_halt_on)
            break;

        // blink stays up for 1s,
        // then down for 500ms.
        if (current_time - initial_time < 1000)
            set_rgb_led(LEDState::ON, ColourIndex::RED);
        else if (current_time - initial_time < 1500)
            set_rgb_led(LEDState::OFF);
        else {
            delay_with_update(500);
            initial_time = current_time;
        }
    }
}

// [FUNCTION SECTION - SETUP HELPERS]

// Function: setup_pin_mode
// Description: Sets all required pin modes for the RGB LED.
// Params: NONE
// Returns: NONE
void setup_pin_mode() noexcept {
    pin_mode(PIN_RGBLED_R, OUTPUT);
    pin_mode(PIN_RGBLED_G, OUTPUT);
    pin_mode(PIN_RGBLED_B, OUTPUT);

    pin_mode(PIN_RELAY_L298N_1, OUTPUT);
    pin_mode(PIN_RELAY_L298N_2, OUTPUT);
    pin_mode(PIN_RELAY_L298N_ENA, OUTPUT);
    pin_mode(PIN_LOAD_CELL_5V, OUTPUT);
    pin_mode(PIN_LOAD_CELL_SCK, OUTPUT);
    pin_mode(PIN_LOAD_CELL_DATA, INPUT);
}

// Function: setup_tngr_handler
// Description: Registers all Thinger.io property handlers.
// Params: NONE
// Returns: NONE
void setup_tngr_handler() noexcept {
    Thing["update_signal"] << handle_tngr_update_signal;
    Thing["manual_feed"]   << handle_tngr_manual_feed_signal;
    Thing["tare_signal"]   << handle_tngr_tare_signal;
}

// Function: setup_load_cell
// Description: Preparing load cell for later usage.
// Params: NONE
// Returns: NONE
void setup_load_cell() noexcept {
    digital_write(PIN_LOAD_CELL_5V, HIGH);

    LoadCell.begin();

    LoadCell.start(LOAD_CELL_STABILISE_TIME_MS);
    LoadCell.set_calibration_factor(LOAD_CELL_CALIBRATION_FACTOR);
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

    Serial.println("[/] INITIALISING LOAD_CELL...");

    setup_load_cell();

    Serial.println("[/] ESTABLISHING WIFI CONNECTION...");
    if (!is_wifi_connected()) {
        handle_wifi_disconnected();
    }

    Serial.println("[+] WIFI CONNECTED!");
    Serial.println("[/] SETTING UP HANDLERS...");

    setup_tngr_handler();

    Serial.println("[+] INITIALISATION COMPLETED!");

    // Blink to indicate success, then idle LED.
    set_rgb_led(LEDState::BLINK, ColourIndex::GREEN, 2, 100, 100);
    set_rgb_led(LEDState::ON, ColourIndex::CYAN);
}

void loop() {
    if (timestamp_program_initialised_ms == 0)
        timestamp_program_initialised_ms = get_timestamp_ms();

    timestamp_current_program_loop_ms = get_timestamp_ms();

    // Reconnect if Wi-Fi drops.
    if (!is_wifi_connected()) {
        handle_wifi_disconnected();
        return;
    }

    // Keep these guys updated.
    listen_for_tngr_interaction();
    Motor.set_speed(motor_rotation_speed);
    update_load_cell();

    Serial.printf("Current weight: %d\n", get_load_cell_g());

    if (tare_amount_since_boot == 0) {
        handle_never_tared();
        return;
    }

    // Ensure required data is valid before continuing.
    if (!is_data_valid()) {
        handle_data_invalid();

        Serial.println("[!] DATA VALIDATED!");
        return;
    }

    // Stop all operations if emergency halt is active.
    if (is_emergency_halt_on) {
        handle_emergency_halt();
        Serial.println("[!] EMERGENCY HALT EXITTED!");
        return;
    }

    if (is_device_running_for_too_long()) {
        handle_device_running_for_too_long();
    }

    // Run active mode loop.
    if (is_running_automatically) {
        set_rgb_led(LEDState::ON, ColourIndex::WHITE);
        handle_automatic_mode_loop();
        set_rgb_led(LEDState::ON, ColourIndex::WHITE);
    }
    else {
        set_rgb_led(LEDState::ON, ColourIndex::ORANGE);
        handle_manual_mode_loop();
        set_rgb_led(LEDState::ON, ColourIndex::ORANGE);
    }
}

// Wrote with love.
// - 0xA1525A.