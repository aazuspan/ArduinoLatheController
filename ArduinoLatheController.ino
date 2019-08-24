/*
    Name:       ArduinoLatheController.ino
    Created:	7/29/2019 11:53:30 AM
    Author:     Aaron Zuspan
    
    Description: This firmware runs the control circuit of a lathe leadscrew motor. This firmware is responsible for 
    interfacing input controls with output electronics. For thorough documentation, see the project repository at
    https://github.com/aazuspan/ArduinoLatheController.
                    
*/

#include <Servo.h>


//#define DEBUG_ON
// Uncomment this define to ignore all the safety checks I carefully wrote (you idiot)
//#define IM_ AN_IDIOT


// Create the servo object
Servo servo;

// Arduino pin assignments
namespace pins
{
    // Input pin assignment
    const byte input_speed_pot = A0;
    const byte input_turbo_activate_switch = 11;
    const byte input_head_direction_switch = 8;
    const byte input_tail_direction_switch = 10;
    const byte input_head_slow_limit_switch = A1;
    const byte input_tail_slow_limit_switch = A3;
    const byte input_head_stop_limit_switch = 2;
    const byte input_tail_stop_limit_switch = 3;

    // Output pin assignment
    const byte output_servo = 9;
    const byte output_direction = 6;
    const byte output_enable = A5;
    const byte output_head_limit_led = 12;
    const byte output_head_moving_led = 13;
    const byte output_tail_limit_led = 4;
    const byte output_tail_moving_led = 5;
}

// Constant variables
namespace constants
{
    // Maximum starting position (speed) at which the stepper will reliably start from standstill  ***NOTE, NEED TO DETERMINE THE CORRECT VALUE ****
    const byte SERVO_STARTING_SPEED = 100;
    // Maximum position (speed) for the servo (use this to limit how far the speed control can be turned by the servo)
    const byte SERVO_MAX_SPEED = 180;
    // Minimum position (speed) for the servo 
    const byte SERVO_MIN_SPEED = 1;
    // Slow speed activated before hitting the stop limit to ensure consistent stop point ***NOTE, NEED TO DETERMINE THE CORRECT VALUE ****
    const byte SERVO_SLOW_SPEED = 50;
}

// Direction values to pass to stepper driver direction pin
enum DirectionValue
{
    TO_TAIL = LOW, TO_HEAD = HIGH
};


// Stepper motor object to track state and control the leadscrew drive motor
class Motor
{
private:
    // Enable values to pass to stepper driver enable pin
    const byte m_MOTOR_ON = LOW;
    const byte m_MOTOR_OFF = HIGH;
    const byte m_enable_pin;
    // Slow mode is engaged between hitting the slow limit and hitting the stop limit to ensure constant speed when stop is hit
    bool m_slow_mode;
    bool m_running;

public:
    // Object constructor
    Motor(byte enable_pin)
        : m_enable_pin(enable_pin), m_slow_mode(false), m_running(false)
    {}

    // Stop the motor
    void stop()
    {
        // Disable motor
        digitalWrite(m_enable_pin, m_MOTOR_OFF);
        m_running = false;
    }

    // If motor isn't currently running, run it
    void run()
    {
        if (!m_running)
        {
            // If the servo is set above the maximum starting speed position, set that speed before running
            if (servo.read() > constants::SERVO_STARTING_SPEED)
                move_servo_to_start();

            // Once the servo reaches the maximum starting speed position, enable the motor
            digitalWrite(m_enable_pin, m_MOTOR_ON);
            m_running = true;
        }
    }

    // Setter for slow mode
    void set_slow_mode(bool state)
    {
        m_slow_mode = state;
    }

    // Getter for slow mode
    bool is_slow_mode()
    {
        if (m_slow_mode)
            return true;
        else
            return false;
    }

    // Check if the motor state is set to running
    bool is_running()
    {
        if (m_running)
            return true;
        else
            return false;
    }
};


// Create the stepper motor object
Motor stepper(pins::output_enable);


// LED object structure to control limit and movement LEDs
class LED
{
private:
    // Current LED state
    bool m_on = false;
    // Arduino pin
    byte m_pin;

public:
    // Object constructor with default values
    LED(byte pin = 0) : m_pin(pin)
    {
    }

    // Flash LED on and off at a constant rate (default 250ms between)
    void flash(int millis_between_flashes = 250)
    {

        // Last time the LED changed states
        static long last_flash_time = millis();
        // Time when this function is called
        long current_time = millis();

        // If it's time to flash, flash
        if (current_time - last_flash_time > millis_between_flashes)
        {
            // Toggle the LED
            toggle();
            // Update the last flash time
            last_flash_time = millis();
        }
    }

    // Toggle between on and off
    void toggle()
    {
        if (m_on)
            turn_off();
        else
            turn_on();
    }

    void turn_on()
    {
        digitalWrite(m_pin, HIGH);
        m_on = true;
    }

    void turn_off()
    {
        digitalWrite(m_pin, LOW);
        m_on = false;
    }
};


// Switch object that keeps track of input pin and current status
class Switch
{
private:
    // Arduino input pin for the switch
    byte m_pin;

public:
    // Object constructor
    Switch(byte pin=0)
        : m_pin(pin)
    {
    }

    // Check if the switch is hit and return boolean
    bool is_hit()
    {
        if (digitalRead(m_pin) == HIGH)
            return true;
        else
            return false;
    }
};


// Limit switch sub-class to control headstock and tailstock limit switches
class LimitSwitch : public Switch
{
private:
    // Switches haven't been checked until the check method is run
    bool checked = false;

    // LED indicator for this limit siwtch
    LED m_led;

public:
    // Object constructor
    LimitSwitch(int pin, LED led=0)
        : Switch(pin), m_led(led)
    {
    }

    // Test if the limit switch works by flashing the LED and having the operator hold the limit. If the switch doesn't work, the program will get stuck in this loop
    void check()
    {
        // Time when the operator hits the limit switch
        long activate_time;
        // Time when the operator releases the limit switch
        long release_time;

        long current_time;
        // How long does the operator have to hold the limit to prove it's working?
        int millis_hold_time = 500;

        // Switch checking loop
        while (!checked)
        {
            m_led.flash();
            // Check if the switch is pressed
            if (is_hit())
            {
                // Get current time when the switch is pressed
                activate_time = millis();

                // Wait for the switch to be released
                while (true)
                {
                    current_time = millis();

                    // If the switch has been held long enough
                    if (current_time - activate_time < millis_hold_time)
                        // Flash fast to signal that switch is held
                        m_led.flash(50);
                    else
                        // Turn the LED off to signal that you can let go
                        m_led.turn_off();

                    // If they let go of the switch or it is intermittent
                    if (!is_hit())
                    {
                        // Get current time when the switch is released
                        release_time = millis();

                        // If the switch was held for long enough to confirm it's working
                        if (release_time - activate_time >= millis_hold_time)
                        {
                            m_led.turn_off();
                            // Good to go
                            checked = true;
                            break;
                        }
                        // If they release the switch too quickly
                        else
                        {
                            // Go back to looking for a switch press
                            break;
                        }
                    }
                }
            }
        }
    }
};


// Direction object (headstock or tailstock) containing the relevant limit switches, LEDs, and direction pins 
class Direction
{
private:
    // Output pin to stepper driver direction
    byte m_output_pin;
    // Value to pass to the stepper driver direction pin when moving
    DirectionValue m_value;

public:
    // Direction switch object that controls movement
    Switch m_direction_switch;
    LimitSwitch m_stop_limit;
    LimitSwitch m_slow_limit;
    LED m_moving_led;
    LED m_limit_led;

    // Object constructor
    Direction(LimitSwitch stop_limit, LimitSwitch slow_limit, LED moving_led, LED limit_led, Switch direction_switch, int output_pin, DirectionValue value)
        : m_stop_limit(stop_limit),
        m_slow_limit(slow_limit),
        m_moving_led(moving_led), 
        m_limit_led(limit_led), 
        m_direction_switch(direction_switch), 
        m_output_pin(output_pin), 
        m_value(value)
    {
    }

    // Turn the limit LED on or off based on switch state
    void update_limit_led()
    {
        if (m_stop_limit.is_hit())
            m_limit_led.turn_on();
        else
            m_limit_led.turn_off();
    }

    // Set the stepper driver towards this direction
    void set_direction()
    {
        digitalWrite(m_output_pin, m_value);
    }

    // Check a direction switch and move that direction if limits aren't hit. Return true if that direction switch is enabled.
    bool check(Direction other)
    {
        // If this direction switch is engaged
        if (m_direction_switch.is_hit())
        {
            // If the limit is reached
            if (m_stop_limit.is_hit())
            {
                // Don't crash
                stepper.stop();
                // Turn the moving LED off
                m_moving_led.turn_off();
                // Flash the warning LED
                m_limit_led.flash(100);
                // We've hit the stop limit, so we're not in slow mode anymore
                stepper.set_slow_mode(false);
            }

            // If the limit hasn't been reached
            else
            {
                // Set the stepper driver direction
                set_direction();
                // Turn on the moving LED
                m_moving_led.turn_on();
                // Turn off the limit LED in case it was still on
                m_limit_led.turn_off();
                // Go!
                stepper.run();

                // If the opposite limit isn't hit, turn that LED off (for example, when moving off of that limit switch)
                if (!other.m_stop_limit.is_hit())
                    other.m_limit_led.turn_off();

                // If the slow limit has been hit but the stop limit hasn't been
                if (m_slow_limit.is_hit())
                    // Engage slow mode
                    stepper.set_slow_mode(true);
                // If the slow limit isn't hit
                else
                    // Disengage slow mode
                    stepper.set_slow_mode(false);
            }
            return true;
        }
        else
            return false;
    }
};


// Create the LED objects
LED head_limit_led(pins::output_head_limit_led);
LED tail_limit_led(pins::output_tail_limit_led);
LED head_moving_led(pins::output_head_moving_led);
LED tail_moving_led(pins::output_tail_moving_led);

// Create the turbo and direction switch objects
Switch turbo_switch(pins::input_turbo_activate_switch);
Switch head_direction_switch(pins::input_head_direction_switch);
Switch tail_direction_switch(pins::input_tail_direction_switch);

// Create the limit switch objects
LimitSwitch head_stop_limit_switch(pins::input_head_stop_limit_switch, head_limit_led);
LimitSwitch tail_stop_limit_switch(pins::input_tail_stop_limit_switch, tail_limit_led);
LimitSwitch head_slow_limit_switch(pins::input_head_slow_limit_switch, head_limit_led);
LimitSwitch tail_slow_limit_switch(pins::input_tail_slow_limit_switch, tail_limit_led);

// Create the direction objects
Direction headstock(head_stop_limit_switch, head_slow_limit_switch, head_moving_led, head_limit_led, head_direction_switch, pins::output_direction, TO_HEAD);
Direction tailstock(tail_stop_limit_switch, tail_slow_limit_switch, tail_moving_led, tail_limit_led, tail_direction_switch, pins::output_direction, TO_TAIL);


// The setup() function runs once each time the micro-controller starts
void setup()
{
    // Begin with motor stopped
    stepper.stop();

    // Set input and output pin modes on Arduino
    set_pin_modes();

    // Attach servo to output servo pin
    servo.attach(pins::output_servo);

    #ifdef DEBUG_ON
        Serial.begin(115200);
    #endif

    #ifndef IM_AN_IDIOT
        // Check that the limit switches are still connected correctly (unless you're an idiot)
        headstock.m_stop_limit.check();
        tailstock.m_stop_limit.check();
    #endif
}


// Set arduino pins to input / output at setup
void set_pin_modes()
{
    // Input pins
    pinMode(pins::input_speed_pot, INPUT);
    pinMode(pins::input_turbo_activate_switch, INPUT);
    pinMode(pins::input_head_direction_switch, INPUT);
    pinMode(pins::input_tail_direction_switch, INPUT);
    pinMode(pins::input_head_stop_limit_switch, INPUT);
    pinMode(pins::input_tail_stop_limit_switch, INPUT);
    pinMode(pins::input_head_slow_limit_switch, INPUT);
    pinMode(pins::input_tail_slow_limit_switch, INPUT);

    // Output pins
    pinMode(pins::output_servo, OUTPUT);
    pinMode(pins::output_direction, OUTPUT);
    pinMode(pins::output_enable, OUTPUT);
    pinMode(pins::output_head_limit_led, OUTPUT);
    pinMode(pins::output_tail_limit_led, OUTPUT);
    pinMode(pins::output_head_moving_led, OUTPUT);
    pinMode(pins::output_tail_moving_led, OUTPUT);
}


// Move servo to the maximum starting speed
void move_servo_to_start()
{
    // Move the servo to the maximum speed where it won't stall
    servo.write(constants::SERVO_STARTING_SPEED);

    // Wait for it to get there
    delay(500);
}


// Move servo to the maximum speed (turbo mode)
void move_servo_to_max()
{
    // zoom zoom
    servo.write(constants::SERVO_MAX_SPEED);
}

// Move servo to the fixed slow speed before hitting the stop limit
void move_servo_to_slow()
{
    servo.write(constants::SERVO_SLOW_SPEED);
}


// Check direction switch position and move accordingly if limits aren't hit
void update_direction()
{
    // Check if we should move towards headstock and do it
    if (!headstock.check(tailstock))
    {
        // If we didn't, check if we should move towards tailstock and do it
        if (!tailstock.check(headstock))
        {
            // If we didn't move towards headstock or tailstock, direction switch must be in middle position
            if (stepper.is_running())
            {
                // Stop
                stepper.stop();
                // Turn moving LEDs off
                headstock.m_moving_led.turn_off();
                tailstock.m_moving_led.turn_off();
            }
            // Turn on/off the appropriate limit LEDs
            headstock.update_limit_led();
            tailstock.update_limit_led();
        }
    }
}


// Get the current pot position and move the servo to it
void update_servo()
{
    // Read the input position from the pot
    int input_position = analogRead(pins::input_speed_pot);

    // Map the input range to the output range
    byte output_position = map(input_position, 0, 1023, constants::SERVO_MIN_SPEED, constants::SERVO_MAX_SPEED);
    // Move the servo
    servo.write(output_position);
}


// Print debugging info to console if DEBUG is defined at beginning
void debug_print()
{
    Serial.print("Motor running: "); Serial.println(stepper.is_running());
    Serial.print("Servo position: "); Serial.println(servo.read());
    Serial.print("Headstock limit switch: "); Serial.println(headstock.m_stop_limit.is_hit());
    Serial.print("Tailtock limit switch: "); Serial.println(tailstock.m_stop_limit.is_hit());
    Serial.print("Moving towards headstock: "); Serial.println(headstock.m_direction_switch.is_hit());
    Serial.print("Moving towards tailstock: "); Serial.println(tailstock.m_direction_switch.is_hit());
    Serial.print("\n\n");
}


// If things that shouldn't happen, happen
bool is_error()
{
    debug_print();
    // Schrodinger's Cat-esque quantum switch state (or a short circuit) 
    if (headstock.m_direction_switch.is_hit() && tailstock.m_direction_switch.is_hit())
    {
#ifdef DEBUG_ON
        debug_print();
        Serial.println("ERROR: Both direction switches pressed.");
#endif 
        return true;
    }


    // We're moving, but we're not sure which direction
//    else if (motor_running && !is_moving_to_head() && !is_moving_to_tail())
//    {
//#ifdef DEBUG_ON
//        debug_print();
//        Serial.println("ERROR: Motor running and not moving either direction.");
//#endif 
//        return true;
//    }
    else
        return false;
}


// Bad things happened. Stop running and freeze until we figure out what went wrong
void protection_mode()
{
    stepper.stop();

    // Fix me. 
    while (true)
    {
        headstock.m_limit_led.flash();
        tailstock.m_limit_led.flash();
    }
}


// Main loop
void loop()
{
    // Check direction switch and move if it's set and limits are clear
    update_direction();

    // If the stepper isn't in slow mode (slow limit isn't hit)
    if (!stepper.is_slow_mode())
    {
        // If turbo is currently held and motor is running
        if (turbo_switch.is_hit() && stepper.is_running())
            // Gotta go fast
            move_servo_to_max();
        else
            // Update the servo position to the speed pot 
            update_servo();
    }
    // If stepper is in slow mode (slow limit is hit but stop limit isn't yet)
    else
    {
        // Run at slow, fixed speed, ignoring speed pot
        move_servo_to_slow();
    }


    #ifndef IM_AN_IDIOT
    // Unless you're an idiot (defined at top of script), check for possible bugs and errors
    if (is_error())
        protection_mode();
    #endif

    #ifdef DEBUG_ON
    // Only print every this many runs to save processing time
    const int print_interval = 5000;
    static int counter = print_interval;
    if (counter > 0)
        counter--;
    else
    {
        // If you're in debugging mode (defined at top of script), print debugging info
        debug_print();
        // Reset the counter
        counter = print_interval;
    }

    #endif
}
