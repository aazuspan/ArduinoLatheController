// South Bend lathe leadscrew control firmware.
// 
/*
    Name:       ArduinoLatheController.ino
    Created:	7/29/2019 11:53:30 AM
    Author:     Aaron Zuspan
*/

#include <Servo.h>


//#define DEBUG_ON
// Uncomment this define to ignore all the safety checks I carefully wrote (you idiot)
//#define IM_ AN_IDIOT


// LED object structure to control limit and movement LEDs
struct LED
{
    // Current LED state
    bool on = false;
    // Arduino pin
    int pin;

    // Object constructor with default values
    LED(int a = 0)
    {
        pin = a;
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
        if (on)
            turn_off();
        else
            turn_on();
    }

    void turn_on()
    {
        digitalWrite(pin, HIGH);
        on = true;
    }

    void turn_off()
    {
        digitalWrite(pin, LOW);
        on = false;
    }
};


// Limit switch object to control headstock and tailstock limit switches
struct LimitSwitch
{
    // Switches haven't been checked until the check method is run
    bool checked = false;
    // Arduino pin
    int pin;
    // LED indicator for this limit siwtch
    LED led;

    // Object constructor
    LimitSwitch(LED a=0, int b=0)
    {
        pin = b;
        led = a;
    }

    // Check if the switch is hit and return boolean
    bool is_hit()
    {
        if (digitalRead(pin) == HIGH)
            return true;
        else
            return false;
    }

    // Turn the limit LED on or off based on switch state
    void update_led()
    {
        if (is_hit())
            led.turn_on();
        else
            led.turn_off();
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
            led.flash();
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
                        led.flash(50);
                    else
                        // Turn the LED off to signal that you can let go
                        led.turn_off();

                    // If they let go of the switch or it is intermittent
                    if (!is_hit())
                    {
                        // Get current time when the switch is released
                        release_time = millis();

                        // If the switch was held for long enough to confirm it's working
                        if (release_time - activate_time >= millis_hold_time)
                        {
                            led.turn_off();
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
struct Direction
{
    LimitSwitch limit;
    LED moving_led;
    LED limit_led;
    // Input pin from direction switch
    int input_pin;
    // Output pin to stepper driver direction
    int output_pin;
    // Value to pass to the stepper driver direction pin when moving
    DirectionValue value;

    // Object constructor
    Direction(LimitSwitch a, LED b, LED c, int d, int e, int f)
    {
        limit = a;
        moving_led = b;
        limit_led = c;
        input_pin = d;
        output_pin = e;
        value = f;
    }

    // Check if the direction switch is set in this direction
    bool is_moving_towards()
    {
        if (digitalRead(input_pin) == HIGH)
            return true;
        else
            return false;
    }

    // Set the stepper driver towards this direction
    void set_direction()
    {
        digitalWrite(output_pin, value);
    }

    // Check a direction switch and move that direction if limits aren't hit. Return true if that direction switch is enabled.
    bool check(Direction other)
    {
        // If direction switch is hit
        if (is_moving_towards())
        {
            // If the limit is reached
            if (limit.is_hit())
            {
                // Don't crash
                stop_motor();
                // Turn the moving LED off
                moving_led.turn_off();
                // Flash the warning LED
                limit_led.flash(100);
            }

            // If the limit hasn't been reached
            else
            {
                // Set the stepper driver direction
                set_direction();
                // Turn on the moving LED
                moving_led.turn_on();
                // Turn off the limit LED in case it was still on
                limit_led.turn_off();
                // Go!
                run_motor();

                // If the opposite limit isn't hit, turn that LED off (for example, when moving off of that limit switch)
                if (!other.limit.is_hit())
                    other.limit_led.turn_off();
            }
            return true;
        }
        else
            return false;
    }
};


// Input pin assignment
const int input_speed_pot = A0;
const int input_turbo_activate_switch = 11;
const int input_head_direction_switch = 8;
const int input_tail_direction_switch = 10;
const int input_head_limit_switch = 12;
const int input_tail_limit_switch = 13;

// Output pin assignment
const int output_servo = 9;
const int output_direction = 6;
const int output_enable = 7;
const int output_head_limit_led = 2;
const int output_tail_limit_led = 4;
const int output_head_moving_led = 3;
const int output_tail_moving_led = 5;

// Direction values to pass to stepper driver direction pin
enum DirectionValue
{
    TO_TAIL = HIGH, TO_HEAD = LOW
};

// Enable values to pass to stepper driver enable pin
const int MOTOR_ON = LOW;
const int MOTOR_OFF = HIGH;

// Maximum starting position (speed) at which the stepper will reliably start from standstill  ***NOTE, NEED TO DETERMINE THE CORRECT VALUE ****
const int SERVO_STARTING_SPEED = 100;

// Maximum position (speed) for the servo (use this to limit how far the speed control can be turned by the servo)
const int SERVO_MAX_SPEED = 180;
// Minimum position (speed) for the servo 
const int SERVO_MIN_SPEED = 1;

// Is your motor running?
bool motor_running = false;

// Create the servo object
Servo servo;

// Create the LED objects with their pin assignments
LED head_limit_led(output_head_limit_led);
LED tail_limit_led(output_tail_limit_led);
LED head_moving_led(output_head_moving_led);
LED tail_moving_led(output_tail_moving_led);

// Create the limit switch objects with their pin assignments 
LimitSwitch head_limit_switch(head_limit_led, input_head_limit_switch);
LimitSwitch tail_limit_switch(tail_limit_led, input_tail_limit_switch);

// Create the direction objects
Direction headstock(head_limit_switch, head_moving_led, head_limit_led, input_head_direction_switch, output_direction, TO_HEAD);
Direction tailstock(tail_limit_switch, tail_moving_led, tail_limit_led, input_tail_direction_switch, output_direction, TO_TAIL);


// The setup() function runs once each time the micro-controller starts
void setup()
{
    // Begin with motor stopped
    stop_motor();

    // Set input and output pin modes on Arduino
    set_pin_modes();

    // Attach servo to output servo pin
    servo.attach(output_servo);

    #ifdef DEBUG_ON
        Serial.begin(115200);
    #endif

    #ifndef IM_AN_IDIOT
        // Check that the limit switches are still connected correctly (unless you're an idiot)
        headstock.limit.check();
        tailstock.limit.check();
    #endif
}


// Set arduino pins to input / output at setup
void set_pin_modes()
{
    // Input pins
    pinMode(input_speed_pot, INPUT);
    pinMode(input_turbo_activate_switch, INPUT);
    pinMode(input_head_direction_switch, INPUT);
    pinMode(input_tail_direction_switch, INPUT);
    pinMode(input_head_limit_switch, INPUT);
    pinMode(input_tail_limit_switch, INPUT);

    // Output pins
    pinMode(output_servo, OUTPUT);
    pinMode(output_direction, OUTPUT);
    pinMode(output_enable, OUTPUT);
    pinMode(output_head_limit_led, OUTPUT);
    pinMode(output_tail_limit_led, OUTPUT);
    pinMode(output_head_moving_led, OUTPUT);
    pinMode(output_tail_moving_led, OUTPUT);
}


// Check if the turbo switch is currrently pressed
bool turbo_engaged()
{
    if (digitalRead(input_turbo_activate_switch) == HIGH)
        return true;
    else
        return false;
}


// Stop the stepper motor
void stop_motor()
{
    // Disable motor
    digitalWrite(output_enable, MOTOR_OFF);
    motor_running = false;

    // Turn moving LEDs off
    headstock.moving_led.turn_off();
    tailstock.moving_led.turn_off();
}


// If motor isn't currently running, run it
void run_motor()
{
    if (!motor_running)
    {
        // If the servo is set above the maximum starting speed position, set that speed before running
        if (servo.read() > SERVO_STARTING_SPEED)
            move_servo_to_start();

        // Once the servo reaches the maximum starting speed position, enable the motor
        digitalWrite(output_enable, MOTOR_ON);
        motor_running = true;
    }
}


// Move servo to the maximum starting speed
void move_servo_to_start()
{
    // Move the servo to the maximum speed where it won't stall
    servo.write(SERVO_STARTING_SPEED);

    // Wait for it to get there
    delay(500);
}


// Move servo to the maximum speed (turbo mode)
void move_servo_to_max()
{
    // zoom zoom
    servo.write(SERVO_MAX_SPEED);
    delay(15);
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
            if (motor_running)
                // Stop
                stop_motor();

            // Turn on/off the appropriate limit LEDs
            headstock.limit.update_led();
            tailstock.limit.update_led();
        }
    }



}


// Get the current pot position and move the servo to it
void update_servo()
{
    // Read the input position from the pot
    int input_position = analogRead(input_speed_pot);

    // Map the input range to the output range
    int output_position = map(input_position, 0, 1023, SERVO_MIN_SPEED, SERVO_MAX_SPEED);
    // Move the servo
    servo.write(output_position);
}


// Print debugging info to console if DEBUG is defined at beginning
void debug_print()
{
    Serial.print("Motor running: "); Serial.println(motor_running);
    Serial.print("Servo position: "); Serial.println(servo.read());
    Serial.print("Headstock limit switch: "); Serial.println(headstock.limit.is_hit());
    Serial.print("Tailtock limit switch: "); Serial.println(tailstock.limit.is_hit());
    Serial.print("Moving towards headstock: "); Serial.println(headstock.is_moving_towards());
    Serial.print("Moving towards tailstock: "); Serial.println(tailstock.is_moving_towards());
    Serial.print("\n\n");
}


// If things that shouldn't happen, happen
bool is_error()
{
    debug_print();
    // Schrodinger's Cat-esque quantum switch state (or a short circuit) 
    if (headstock.is_moving_towards() && tailstock.is_moving_towards())
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
    stop_motor();

    // Fix me. 
    while (true)
    {
        headstock.limit_led.flash();
        tailstock.limit_led.flash();
    }
}


// Main loop
void loop()
{
    // Check direction switch and move if it's set and limits are clear
    update_direction();

    // If turbo is currently held and motor is running
    if (turbo_engaged() && motor_running)
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
