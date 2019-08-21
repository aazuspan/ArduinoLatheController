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
    const byte input_head_limit_switch = 2;
    const byte input_tail_limit_switch = 3;
    const byte input_enable = A5;

    // Output pin assignment
    const byte output_servo = 9;
    const byte output_direction = 6;
    const byte output_enable = A3;
    const byte output_tail_relay = A1;
    const byte output_head_relay = 7;
    const byte output_head_limit_led = 12;
    const byte output_tail_limit_led = 4;
    const byte output_head_moving_led = 13;
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
}

// Enable value to enable bypass relay. When output is HIGH, relay is triggered, sending 5v to enable, disabling the stepper
enum EnableValue
{
    GO = LOW, STOP = HIGH
};

// Relay values to pass to the enable bypass relays
enum BypassValue
{
    ON = LOW, OFF = HIGH
};

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
    const byte m_enable_output;
    const byte m_enable_input;

public:
    bool m_running;

    // Object constructor
    Motor(byte output_enable, byte input_enable)
        : m_enable_output(output_enable), m_enable_input(input_enable), m_running(false)
    {
        // Disable the motor
        set_enable_relay(STOP);
    }

    // Stop the motor
    void stop()
    {
        // Disable motor
        set_enable_relay(STOP);
        m_running = false;
    }

    // If motor isn't currently running, run it
    void run()
    {
        // If the servo is set above the maximum starting speed position, set that speed before running
        //if (servo.read() > constants::SERVO_STARTING_SPEED)
        //    move_servo_to_start();

        // Once the servo reaches the maximum starting speed position, enable the motor
        set_enable_relay(GO);
        m_running = true;
    }

    // Check if the stepper driver is enabled, which would mean the motor is running
    bool is_running()
    {
        if (digitalRead(m_enable_input) == LOW)
            return true;
        else
            return false;
    }

private:
    // Set the enable relay state. When the relay is on, 5V will be cut off from the enable pin, allowing the motor to move. This is used to stop the motor when direction switch is in middle position.
    void set_enable_relay(EnableValue state)
    {
        // Set the enable relay output to the correct state
        digitalWrite(m_enable_output, state);
    }
};


// Create the stepper motor object
Motor stepper(pins::output_enable, pins::input_enable);


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

    // Turn the limit LED on or off based on switch state
    void update_led()
    {
        if (is_hit())
            m_led.turn_on();
        else
            m_led.turn_off();
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


// Direction object (headstock or tailstock) containing the relevant limit switch, LEDs, and direction pins 
class Direction
{
private:
    // Output pin to stepper driver direction
    byte m_direction_output_pin;
    // Output pin to enable bypass relay
    byte m_relay_output_pin;
    // Output pin to enable relay to stop motor when limit switches aren't hit
    byte m_enable_pin;
    // Value to pass to the stepper driver direction pin when moving
    DirectionValue m_value;

public:
    // Direction switch object that controls movement
    Switch m_direction_switch;
    LimitSwitch m_limit;
    LED m_moving_led;
    LED m_limit_led;

    // Object constructor
    Direction(LimitSwitch limit, LED moving_led, LED limit_led, Switch direction_switch, int direction_output_pin, int relay_output_pin, DirectionValue value)
        : m_limit(limit), 
        m_moving_led(moving_led), 
        m_limit_led(limit_led), 
        m_direction_switch(direction_switch), 
        m_direction_output_pin(direction_output_pin), 
        m_relay_output_pin(relay_output_pin),
        m_value(value)
    {
        // Start be disabling the bypass relay
        set_bypass_relay(OFF);
    }

    // Set the stepper driver towards this direction
    void set_direction()
    {
        digitalWrite(m_direction_output_pin, m_value);
    }

    // Set the bypass relay state. When bypass is on, the relay bypasses the limit switch that disables the motor. This is used when moving away from a limit switch that is hit
    void set_bypass_relay(BypassValue state)
    {
        // Set the relay bypass output to the correct state
        digitalWrite(m_relay_output_pin, state);
    }

    // Check a direction switch and move that direction if limits aren't hit. Return true if that direction switch is enabled.
    bool check(Direction other)
    {
        // If this direction switch is engaged
        if (m_direction_switch.is_hit())
        {
            // If this limit is reached, update LEDs
            if (m_limit.is_hit())
            {
                stepper.m_running = false;
                // Turn the moving LED off
                m_moving_led.turn_off();
                // Flash the warning LED
                m_limit_led.flash(100);
            }

            // If the opposite limit is hit
            else if (other.m_limit.is_hit())
            {
                // Bypass the limit switch to enable the stepper to run away from that limit
                set_bypass_relay(ON);
                stepper.m_running = true;
                m_moving_led.turn_on();
                set_direction();
            }


            // If the limit hasn't been reached
            else
            {
                stepper.m_running = true;
                // Disable the bypass relay to allow the limit switch to disable the motor when hit
                set_bypass_relay(OFF);
                // Set the stepper driver direction
                set_direction();
                // Turn on the moving LED
                m_moving_led.turn_on();
                // Turn off the limit LED in case it was still on
                m_limit_led.turn_off();
                // Since the opposite limit isn't hit, turn that LED off (for example, when moving off of that limit switch)
                other.m_limit_led.turn_off();
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
LimitSwitch head_limit_switch(pins::input_head_limit_switch, head_limit_led);
LimitSwitch tail_limit_switch(pins::input_tail_limit_switch, tail_limit_led);

// Create the direction objects
Direction headstock(head_limit_switch, head_moving_led, head_limit_led, head_direction_switch, pins::output_direction, pins::output_head_relay, TO_HEAD);
Direction tailstock(tail_limit_switch, tail_moving_led, tail_limit_led, tail_direction_switch, pins::output_direction, pins::output_tail_relay, TO_TAIL);


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
        headstock.m_limit.check();
        tailstock.m_limit.check();
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
    pinMode(pins::input_head_limit_switch, INPUT);
    pinMode(pins::input_tail_limit_switch, INPUT);

    // Output pins
    pinMode(pins::output_servo, OUTPUT);
    pinMode(pins::output_direction, OUTPUT);
    pinMode(pins::output_enable, OUTPUT);
    pinMode(pins::output_head_relay, OUTPUT);
    pinMode(pins::output_tail_relay, OUTPUT);
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
                // Set the enable relay to stop the motor
                stepper.stop();

                // Turn moving LEDs off
                headstock.m_moving_led.turn_off();
                tailstock.m_moving_led.turn_off();
            }

            // Turn on/off the appropriate limit LEDs
            headstock.m_limit.update_led();
            tailstock.m_limit.update_led();
        }
        // Tailstock switch is set
        else
        {
            // Set the enable relay to allow the motor to go
            stepper.run();
        }
    }
    // Headstock switch is set
    else
    {
        // Set the enable relay to allow the motor to go
        stepper.run();
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
    Serial.print("Headstock limit switch: "); Serial.println(headstock.m_limit.is_hit());
    Serial.print("Tailtock limit switch: "); Serial.println(tailstock.m_limit.is_hit());
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

    // If turbo is currently held and motor is running
    if (turbo_switch.is_hit() && stepper.is_running())
        // Gotta go fast
        move_servo_to_max();
    else
        // Update the servo position to the speed pot 
        update_servo();

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
