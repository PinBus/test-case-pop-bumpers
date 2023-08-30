/**
 * Test-Case - Pop Bumpers
 *
 * Version 1.0 (01-12-2021)
 * Version 1.1 (24-08-2023)
 **/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/** Define Arduino pins and PCA9685 pins **/
#define PIN_LED_HEARTBEAT 9

#define PIN_MATRIX_IN_0 A0
#define PIN_MATRIX_IN_1 A1
#define PIN_MATRIX_IN_2 A2
#define PIN_MATRIX_IN_3 A3

#define PIN_MATRIX_OUT_0 4
#define PIN_MATRIX_OUT_1 5
#define PIN_MATRIX_OUT_2 6
#define PIN_MATRIX_OUT_3 7

#define PIN_PCA9685_OE 8

//  0 - Solenoid: Flipper Power Winding
//  1 - Solenoid: Flipper Full Winding
//  2 - Solenoid: Pop Bumper Red
//  3 - Solenoid: Pop Bumper Yellow
//  4 - Solenoid: Pop Bumper Blue
//  5 - LED: Unused
//  6 - LED: Unused
//  7 - LED: Unused
#define PIN_PCA9685_FLIPPER_POWER 0
#define PIN_PCA9685_FLIPPER_HOLD 1
#define PIN_PCA9685_POP_BUMPER_RED 2
#define PIN_PCA9685_POP_BUMPER_YELLOW 3
#define PIN_PCA9685_POP_BUMPER_BLUE 4

//  8 - LED: Flipper Button
//  9 - LED: Pop Bumper Button Red
// 10 - LED: Pop Bumper Button Yellow
// 11 - LED: Pop Bumper Button Blue
// 12 - LED: Unused
// 13 - LED: Unused
// 14 - LED: Unused
// 15 - LED: Unused
#define PIN_PCA9685_LED_BUTTON_FLIPPER 8
#define PIN_PCA9685_LED_BUTTON_RED 9
#define PIN_PCA9685_LED_BUTTON_YELLOW 10
#define PIN_PCA9685_LED_BUTTON_BLUE 11

/** Type definitions **/
typedef void (*SwitchCallback)();

struct SwitchState {
    uint32_t first_change;
    bool is_closed;
    SwitchCallback on_switch_close;
    SwitchCallback on_switch_open;
};

enum LedAnimationState {
    Running = 0,
    Pauzed = 1,
    FadeIn = 2,
    FadeOut = 3
};

enum LedAnimationType {
    Static = 0,
    Fade = 1
};

struct LedAnimationStep {
    LedAnimationType type;
    uint32_t duration;
    uint16_t pwm_value;
};

struct LedAnimation {
    LedAnimationStep* steps;
    uint32_t last_update;
    uint8_t current_step;
    uint16_t current_pwm_value;
    uint16_t backup_pwm_value;
    uint8_t number_of_steps;
    uint8_t pin;
    LedAnimationState state;
};

struct Flipper {
    uint8_t power_winding_pin;
    uint8_t hold_winding_pin;
    uint32_t active_time;
    bool is_active;
};

struct PopBumper {
    uint8_t pin;
    uint8_t led_pin;
    uint32_t timer;
};

/** Global variables **/

// The PCA9685 driver object
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

/** Configuration **/
#define MATRIX_DEBOUNCE_MS 20
#define MATRIX_HEIGHT 4
#define MATRIX_WIDTH 4

#define FLIPPER_POWER_WINDING_MAX_MS 1000
#define FLIPPER_POWER_WINDING_POWER_PWM 4096
#define FLIPPER_HOLD_WINDING_POWER_PWM 4096

#define POP_BUMPER_MS 30

#define LED_FADE_SPEED_MS 200
#define LED_MAX_VALUE 4096
#define LED_HALF_VALUE 2048
#define LED_QUARTER_VALUE 1024

/** Heartbeat LED **/
uint8_t heartbeat_led_index = 0;
uint32_t heartbeat_led_sequence[4] = { 200, 125, 200, 600 };
uint32_t heartbeat_led_update = 0;
bool heartbeat_led_state = true;

void init_heartbeat_led() {
    // Set up the heartbeat LED pin
    pinMode(PIN_LED_HEARTBEAT, OUTPUT);
    digitalWrite(PIN_LED_HEARTBEAT, heartbeat_led_state);
}

void update_heartbeat_led() {
    if (millis() - heartbeat_led_update < heartbeat_led_sequence[heartbeat_led_index]) {
        // Nothing to update yet
        return;
    }

    // Increase the index and make sure it does not go out of bounds
    heartbeat_led_index++;
    if (heartbeat_led_index == 4) {
        heartbeat_led_index = 0;
    }

    // Mark now as the last update
    heartbeat_led_update = millis();

    // Update the pin state and flip the internal state
    digitalWrite(PIN_LED_HEARTBEAT, heartbeat_led_state);
    heartbeat_led_state = !heartbeat_led_state;
}

/** Switch Matrix **/
uint8_t matrix_input_pins[MATRIX_HEIGHT] = {
    PIN_MATRIX_IN_0,
    PIN_MATRIX_IN_1,
    PIN_MATRIX_IN_2,
    PIN_MATRIX_IN_3
};
uint8_t matrix_output_pins[MATRIX_WIDTH] = {
    PIN_MATRIX_OUT_0,
    PIN_MATRIX_OUT_1,
    PIN_MATRIX_OUT_2,
    PIN_MATRIX_OUT_3
};

uint8_t matrix_col = 0;
uint8_t matrix_row = 0;

// Scratchpad pointer for loops iterating over switch state structs
SwitchState* switch_state = NULL;

SwitchState matrix[MATRIX_WIDTH][MATRIX_HEIGHT] = {
    {
        // 0x0 - NO - Front button 1: White / Flipper
        {0, false, on_flipper_button_down, on_flipper_button_up},
        // 0x1 - NO - Front button 2: Red / Pop Bumper
        {0, false, on_bumper_button_red_down, NULL},
        // 0x2 - NO - Front button 3: Yellow / Pop Bumper
        {0, false, on_bumper_button_yellow_down, NULL},
        // 0x3 - NO - Front button 4: Blue / Pop Bumper
        {0, false, on_bumper_button_blue_down, NULL}
    },
    {
        // 1x0 - NO - Unused
        {0, false, NULL, NULL},
        // 1x1 - NO - Unused
        {0, false, NULL, NULL},
        // 1x2 - NO - Unused
        {0, false, NULL, NULL},
        // 1x3 - NO - Unused
        {0, false, NULL, NULL}
    },
    {
        // 2x0 - NO - Pop Bumper Red
        {0, false, on_bumper_red_closed, NULL},
        // 2x1 - NO - Pop Bumper Yellow
        {0, false, on_bumper_yellow_closed, NULL},
        // 2x2 - NO - Pop Bumper Blue
        {0, false, on_bumper_blue_closed, NULL},
        // 2x3 - NC - Flipper EOS
        {0, false, NULL, on_flipper_eos_open}
    },
    {
        // 3x0 - NO - Unused
        {0, false, NULL, NULL},
        // 3x1 - NO - Unused
        {0, false, NULL, NULL},
        // 3x2 - NO - Unused
        {0, false, NULL, NULL},
        // 3x3 - NO - Unused
        {0, false, NULL, NULL}
    }
};

void init_matrix() {
    // Set pin mode for output pins
    for (matrix_col = 0; matrix_col < MATRIX_WIDTH; matrix_col++) {
        pinMode(matrix_output_pins[matrix_col], OUTPUT);
    }

    // Set pin mode for input pins
    for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
        pinMode(matrix_input_pins[matrix_row], INPUT);
    }
}

void update_matrix() {
    for (matrix_col = 0; matrix_col < MATRIX_WIDTH; matrix_col++) {
        // Set output pins
        for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
            digitalWrite(matrix_output_pins[matrix_row], matrix_col != matrix_row);
        }

        // Scan input pins
        for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
            switch_state = &matrix[matrix_row][matrix_col];

            // Check if switch state has been changed
            if (digitalRead(matrix_input_pins[matrix_row]) == (switch_state->is_closed ? HIGH : LOW)) {
                if (!switch_state->first_change) {
                    // Mark the first time this state change has been seen
                    switch_state->first_change = millis();
                }
                else {
                    // Check if switch is in its new state for long enough
                    if (millis() - switch_state->first_change > MATRIX_DEBOUNCE_MS) {
                        if (switch_state->is_closed) {
                            // Call the on switch open callback
                            if (switch_state->on_switch_open != NULL) {
                                switch_state->on_switch_open();
                            }
                        }
                        else {
                            // Call the on switch close callback
                            if (switch_state->on_switch_close != NULL) {
                                switch_state->on_switch_close();
                            }
                        }

                        // Update internal state
                        switch_state->is_closed = !switch_state->is_closed;
                        switch_state->first_change = 0;
                    }
                }
            }
            else if (millis() - switch_state->first_change > MATRIX_DEBOUNCE_MS) {
                // Reset internal state if switch went back to its previous state within the debounce time
                switch_state->first_change = 0;
            }
        }
    }
}

/** PCA9685 **/
void init_pca9685() {
    // Constructor initializes a PCA9685 device at default address 0x40
    pca9685.begin();
    // Set maximum PWM frequency in Hz
    pca9685.setPWMFreq(1600);
    // Set output to push/pull (totempole)
    pca9685.setOutputMode(true);

    // Set Output Enable (OE) pin low
    pinMode(PIN_PCA9685_OE, OUTPUT);
    digitalWrite(PIN_PCA9685_OE, LOW);
}

/** Solenoid Control **/
Flipper flipper = {
    PIN_PCA9685_FLIPPER_POWER,
    PIN_PCA9685_FLIPPER_HOLD,
    0,
    false
};

void update_flipper() {
    if (!flipper.active_time) {
        // Flipper not active, nothing to update
        return;
    }

    // Power down the power winding after 1 second regardless of the EOS switch state
    if (millis() - flipper.active_time > FLIPPER_POWER_WINDING_MAX_MS) {
        flipper_eos();
    }
}

void flipper_activate() {
    // Update flipper state
    flipper.is_active = true;
    flipper.active_time = millis();

    // Activate the power winding of the flipper coil
    pca9685.setPin(flipper.power_winding_pin, FLIPPER_POWER_WINDING_POWER_PWM, false);
}

void flipper_eos() {
    if (!flipper.is_active) {
        return;
    }

    // Update flipper state
    flipper.active_time = 0;

    // Deactivate power winding and activate hold winding
    pca9685.setPin(PIN_PCA9685_FLIPPER_POWER, 0, false);
    pca9685.setPin(PIN_PCA9685_FLIPPER_HOLD, FLIPPER_HOLD_WINDING_POWER_PWM, false);
}

void flipper_deactivate() {
    // Update flipper state
    flipper.is_active = false;
    flipper.active_time = 0;

    // Power down both flipper coil windings
    pca9685.setPin(PIN_PCA9685_FLIPPER_POWER, 0, false);
    pca9685.setPin(PIN_PCA9685_FLIPPER_HOLD, 0, false);
}

PopBumper pop_bumpers[3] = {
    { PIN_PCA9685_POP_BUMPER_RED, PIN_PCA9685_LED_BUTTON_RED, 0 },
    { PIN_PCA9685_POP_BUMPER_YELLOW, PIN_PCA9685_LED_BUTTON_YELLOW, 0 },
    { PIN_PCA9685_POP_BUMPER_BLUE, PIN_PCA9685_LED_BUTTON_BLUE,  0 }
};

// Scratchpad pointer for loops iterating over pop bumpers
PopBumper* pop_bumper = NULL;

void fire_pop_bumper(uint8_t index) {
    pop_bumper = &pop_bumpers[index];

    // Update state
    pop_bumper->timer = millis();

    // Fire coil and highlight button
    pca9685.setPin(pop_bumper->pin, 4096, false);
    pca9685.setPin(pop_bumper->led_pin, LED_MAX_VALUE, true);
}

void update_pop_bumpers() {
    for (uint8_t i = 0; i < 3; i++) {
        pop_bumper = &pop_bumpers[i];

        if (pop_bumper->timer) {
            if (millis() - pop_bumper->timer > POP_BUMPER_MS) {
                // Update state
                pop_bumper->timer = 0;

                // Disable coil and reset button LED
                pca9685.setPin(pop_bumper->pin, 0, false);
                pca9685.setPin(pop_bumper->led_pin, LED_QUARTER_VALUE, true);
            }
        }
    }
}

void init_leds() {
    pca9685.setPin(PIN_PCA9685_LED_BUTTON_RED, LED_QUARTER_VALUE, true);
    pca9685.setPin(PIN_PCA9685_LED_BUTTON_YELLOW, LED_QUARTER_VALUE, true);
    pca9685.setPin(PIN_PCA9685_LED_BUTTON_BLUE, LED_QUARTER_VALUE, true);
    pca9685.setPin(PIN_PCA9685_LED_BUTTON_FLIPPER, LED_MAX_VALUE, true);
}

/** Serial **/
void init_serial() {
    // Open the serial connection
    Serial.begin(9600);
}

void update_serial() {
    // Check for available characters on the serial port.
    if (Serial.available() > 0) {
        // If the sent character is a "c", print the credits "window"
        if (Serial.read() == 'c') {
            Serial.println("------  Test-Case - Pop Bumpers   ------\n        Version 1.0 (01-12-2021)\n       Made in The Netherlands by\n    Cor Gravekamp & Thomas Gravekamp\n                 for the\n          Dutch Pinball Museum\n----------------------------------------");
        }
    }
}

/** Button action **/
void on_flipper_button_down() {
  flipper_activate();
}

void on_flipper_button_up() {
  flipper_deactivate();
}

void on_flipper_eos_open() {
  flipper_eos();
}

void on_bumper_button_red_down() {
    fire_pop_bumper(0);
}

void on_bumper_button_yellow_down() {
    fire_pop_bumper(1);
}

void on_bumper_button_blue_down() {
    fire_pop_bumper(2);
}

void on_bumper_red_closed() {
    fire_pop_bumper(0);
}

void on_bumper_yellow_closed() {
    fire_pop_bumper(1);
}

void on_bumper_blue_closed() {
    fire_pop_bumper(2);
}

void setup()
{
    // Initialize heartbeat LED
    init_heartbeat_led();

    // Initialize switch matrix
    init_matrix();

    // Initialize PCA9685
    init_pca9685();

    // Initialize LEDs
    init_leds();

    // Initialize serial
    init_serial();
}

void loop()
{
    // Update heartbeat LED
    update_heartbeat_led();

    // Update switch matrix
    update_matrix();

    // Update flipper timer
    update_flipper();

    // Update pop bumpers
    update_pop_bumpers();

    // Update serial
    update_serial();
}
