# ArduinoLatheController
An Arduino-based stepper motor control with servo speed adjustment to run a lathe leadscrew (in progress)

    Name:       ArduinoLatheController.ino
    Created:	7/29/2019 11:53:30 AM
    Author:     Aaron Zuspan
    
    Description: This firmware runs the control circuit of a lathe leadscrew motor. This firmware is responsible for 
    interfacing input controls with output electronics. 
    
    Input controls include a potentiometer that controls motor speed, a momentary switch to engage max speed, a three 
    position direction switch that controls motor direction, and two limit switches used to automatically stop the motor 
    at preset stop positions. 
    
    Output electronics include a stepper motor driver that takes enable, direction, and pulse signals and runs the 
    stepper motor, a stepper motor speed control that outputs variable pulses to the driver based on the position of a 
    potentiometer, and a servo that is controlled by this firmware and sets the position of the speed control 
    potentiometer.

    Speed control: The input speed potentiometer feeds an analog input on the Arduino. This input is mapped to the output
    servo range, and the servo is set to that position. The servo directly drives the potentiometer of the stepper speed
    controller. This servo drive is implemented for two reasons. First, the dedicated PWM driver is capable of providing
    a smoother signal to the stepper driver than the Arduino, and is not interrupted by running other operations. Second,
    using a potentiometer to control a servo to control a potentiometer allows for software governing of speed.

    Speed governing: If the stepper motor is started immediately at high speed, it will stall. To avoid this, speed is 
    checked whenever the motor is started. If the speed is set above the maximum starting speed, the servo will
    automatically reduce to this speed, the motor will start, and the servo will run up to the speed set by the speed
    control.

    Direction: The direction switch feeds two digital inputs, representing directions, to the Arduino. When the switch is 
    set to one of these positions, the Arduino sends the appropriate direction signal to the stepper driver, as well as 
    setting the enable signal to the stepper driver in order to run the stepper motor. If the limit switch corresponding 
    to that direction is hit, the motor will not be allowed to move in that direction. If the direction switch is set to 
    the middle position, it will stop.

    Limit switches: A total of four optical limit switches feed digital inputs to the Arduino. At the tailstock and headstock, 
    a pair of optical limit switches mounted adjacently act to slow the stepper motor to a fixed speed and then stop it. The 
    slowdown limit switch is used to ensure that the stepper motor is always running at the same speed when the stop limit is 
    hit. This guarantees that stop points are repeatable within less than one thousandth of an inch. After testing a number of
    alternate solutions, it was determined that repeatability was not achievable without running the motor at a constant speed
    due to inertia. 

    Turbo: If the motor is running, the turbo switch will set the servo speed to maximum regardless of speed control
    position. This is also subject to speed governing.
