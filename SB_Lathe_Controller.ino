// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       SB_Lathe_Controller.ino
    Created:	7/29/2019 11:53:30 AM
    Author:     DESKTOP-9SDRCVD\Admin
*/

#include <Servo.h>

#define DEBUG


// LED object structure to control limit and movement LEDs
struct LED
{
    bool on;
    int pin;

    // Object constructor
    LED(int a)
    {
        on = false;
        pin = a;
    }

    // Flash LED on and off
    void flash()
    {
        // Flash_interval represents how often the LED changes between on and off. The speed will change based on how often this function is called
        const long int flash_interval = 100000;

        // Counter that decrements to change LED state
        static long int counter = flash_interval;

        // For the first half of the interval, LED is on
        if (counter > flash_interval / 2)
        {
            turn_on();
        }

        // For the second half of the interval, LED is off
        else if (counter > 0)
        {
            turn_off();
        }

        // When the interval is done, reset
        else
            // reset the counter
            counter = flash_interval;

        // Increment every time this function is called
        counter -= 1;
    }

    // If not already on, turn on
    void turn_on()
    {
        if (!on) 
        {
            digitalWrite(pin, HIGH);
            on = true;
        }

    }

    // If on, turn off
    void turn_off()
    {
        if (on)
        {
            digitalWrite(pin, LOW);
            on = false;
        }
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

// Direction values to pass to stepper driver direction pin ***NOTE, THESE MIGHT BE BACKWARDS****
const int TO_TAIL = HIGH;
const int TO_HEAD = LOW;

// Enable values to pass to stepper driver enable pin
const int MOTOR_ON = LOW;
const int MOTOR_OFF = HIGH;

// Maximum starting position (speed) at which the stepper will reliably start from standstill  ***NOTE, NEED TO DETERMINE THE CORRECT VALUE ****
const int SERVO_STARTING_SPEED = 100;

// Maximum position (speed) for the servo (use this to limit the max speed setting)
const int SERVO_MAX_SPEED = 180;

// Used to keep track of whether motor is currently running
bool motor_running = false;

// Create the servo object
Servo servo;

// Create the LED objects with their pin assignments
LED head_limit_led(output_head_limit_led);
LED tail_limit_led(output_tail_limit_led);
LED head_moving_led(output_head_moving_led);
LED tail_moving_led(output_tail_moving_led);


// The setup() function runs once each time the micro-controller starts
void setup()
{
    #ifdef DEBUG
    Serial.begin(9600);
    #endif

    // Set input and output pin modes on Arduino
    set_pin_modes();

    // Attach servo to output servo pin
    servo.attach(output_servo);
}


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

// Check if headstock limit switch has been reached
bool is_at_head()
{
    if (digitalRead(input_head_limit_switch) == HIGH) {
        return true;
    }
    else
        return false;
}


// Check if tailstock limit switch has been reached
bool is_at_tail()
{
    if (digitalRead(input_tail_limit_switch) == HIGH) {
        return true;
    }
    else
        return false;
}


// Check if direction switch is towards head
bool is_moving_to_head()
{
    if (input_head_direction_switch == HIGH)
        return true;
}


// Check if direction switch is towards tail
bool is_moving_to_tail()
{
    if (input_tail_direction_switch == HIGH)
        return true;
}


// Stop the stepper motor
void stop_motor()
{
    // Disable motor
    digitalWrite(output_enable, MOTOR_OFF);
    motor_running = false;

    // Turn moving LEDs off
    head_moving_led.turn_off();
    tail_moving_led.turn_off();

    // If servo is set above the maximum starting speed position, move to it
    if (servo.read() > SERVO_STARTING_SPEED)
        move_servo_to_start();
}


// Move servo to the maximum starting speed
void move_servo_to_start()
{
    // Move the servo
    servo.write(SERVO_STARTING_SPEED);

    // Wait until it reaches that position to release control
    while (servo.read() > SERVO_STARTING_SPEED)
        delay(5);
}


// Move servo to the maximum speed
void move_servo_to_max()
{
    // Move the servo (note: this can be interrupted if speed is changed)
    servo.write(SERVO_MAX_SPEED);
}


// If motor isn't currently running, run it
void run_motor()
{
    if (!motor_running)
    {
        // If the servo is set above the maximum starting speed position, move to it
        if (servo.read() > SERVO_STARTING_SPEED)
            move_servo_to_start();

        // Once the servo reaches the maximum starting speed position, enable the motor
        digitalWrite(output_enable, MOTOR_ON);
        motor_running = true;
    }
}


// Check direction switch position and move accordingly if limits aren't hit
void update_direction()
{
    // If direction switch is towards head
    if (is_moving_to_head)
    {
        // If headstock limit switch is reached
        if (is_at_head)
        {
            // STOP!
            stop_motor();
            // Turn the headstock warning LED on
            head_limit_led.turn_on();
            // Flash the headstock moving LED
            head_moving_led.flash();
        }

        // If head limit switch hasn't been reached
        else
        {
            // Set stepper direction towards head
            digitalWrite(output_direction, TO_HEAD);
            // Turn the 'moving towards head' LED on
            head_moving_led.turn_on();
            // Turn the headstock warning LED off
            head_limit_led.turn_off();
            // Run the motor
            run_motor();
        }
    }

    // If direction switch is towards tail
    else if (is_moving_to_tail)
    {
        // If tailstock limit switch is reached
        if (is_at_tail)
        {
            // STOP!
            stop_motor();
            // Turn the tailstock warning LED on
            tail_limit_led.turn_on();
            // Flash the tailstock moving LED
            tail_moving_led.flash();
        }
        
        // If tail limit switch hasn't been reached
        else
        {
            // Set stepper direction towards tail
            digitalWrite(output_direction, TO_TAIL);
            // Turn the 'moving towards tail' LED on
            tail_moving_led.turn_on();
            // Turn the tailstock warning LED off
            tail_limit_led.turn_off();
            // Run the motor
            run_motor();
        }
    }

    // If direction switch is in middle position
    else
        // Stop
        stop_motor();
}


// Get the current pot position and move the servo to it
void update_servo()
{
    // Read the input position from the pot
    int input_position = analogRead(input_speed_pot);
    // Map the input range to the output range
    int output_position = map(input_position, 0, 1023, 0, 180);
    // Move the servo
    servo.write(output_position);
}


// Check if the turbo switch is currrently pressed
bool turbo_engaged()
{
    if (digitalRead(input_turbo_activate_switch == HIGH))
        return true;
}

// Print debugging info to console if DEBUG is defined at beginning
void debug_print()
{
    // Only print every this many runs to save processing time
    const int print_interval = 5000;
    static int counter = print_interval;
    if (counter > 0)
        counter--;
    else
    {
        Serial.print("Motor running: "); Serial.println(motor_running);
        Serial.print("Servo position: "); Serial.println(servo.read());
        Serial.print("Headstock limit switch: "); Serial.println(is_at_head());
        Serial.print("Tailtock limit switch: "); Serial.println(is_at_tail());
        Serial.print("Moving towards headstock: "); Serial.println(is_moving_to_head());
        Serial.print("Moving towards tailstock: "); Serial.println(is_moving_to_tail());
        Serial.print("\n\n");
        // Reset counter
        counter = print_interval;
    }
}

// Main loop
void loop()
{
    // Check direction switch and move if set and limits are clear
    update_direction();

    // If turbo is currently held
    if (turbo_engaged)
        // Move the speed to max
        move_servo_to_max();
    else
        // Update the servo position to the speed pot 
        update_servo();

    #ifdef DEBUG
    // If you're in debugging mode (defined at top of script), print debugging info
    debug_print();
    #endif
}
