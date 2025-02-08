#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

// left motor group
pros::MotorGroup left_motor_group({-4, -20, -6}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({1, 2, 3}, pros::MotorGears::blue);
pros::adi::DigitalOut piston('A');
pros::adi::DigitalOut doinker('B');


// Intake motors
pros::Motor intake_lower(7);
pros::Motor intake_upper(8);
pros::Motor arm(9);

//Auton paths
ASSET(TEST_txt);

//Sensors
pros::Rotation rotationSensor(12);



// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              14, // 14 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(10);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

//Arm Mech
const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0, 300, 2000};
int currState = 0;
int target = 0;

/**
 * @brief Changes the state of the arm
 * @cite 8pxl 
 */
void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}


/**
 * @brief  Controls the arm motion
 * @cite 8pxl
 */
void liftControl() {
    double kp = 0.5;
    double error = target - rotationSensor.get_position();
    double velocity = kp * error;
    arm.move(velocity);
}


void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    rotationSensor.reset_position();

    // print position to brain screen

    //Task for arm
    pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });

    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
			pros::lcd::print(3, "Arm Theta: %f", rotationSensor.get_angle()); // heading
			//
			//-2610 - max
            // delay to save resources
            pros::delay(20);
        }
    });

}

void disabled() {}

void competition_initialize() {}

void moveIntakeforward()
{
    int lowerIntakePower = 127;
    int upperIntakePower = lowerIntakePower;
    intake_lower.move(lowerIntakePower);
    intake_upper.move(upperIntakePower);
}
void stopIntake()
{
    intake_lower.move(0);
    intake_upper.move(0);
}
void moveIntakebackword() {
    int lowerIntakePower = 127;
    int upperIntakePower = lowerIntakePower;
    intake_lower.move(-1*lowerIntakePower);
    intake_upper.move(-1*upperIntakePower);
}

void pathJerry()
{

    chassis.follow(TEST_txt, 15, 1000);
}

void redLeft()
{
          chassis.setPose(0,0,180);
    chassis.moveToPoint(0,  40, 2000, {.forwards = false,.maxSpeed = 70,.minSpeed = 10});
    chassis.waitUntilDone();        
    pros::delay(600);
    piston.set_value(true);
    pros::delay(1500);
    moveIntakeforward();
    chassis.turnToPoint(-29, 35, 1000);
    chassis.moveToPoint(-29, 35, 3000);
    chassis.turnToPoint(-31, 47, 1000);
    chassis.moveToPoint(-31, 47,2000,{.forwards = true,.maxSpeed =  90,.minSpeed = 10});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(-31, 37,2000,{.forwards = false,.maxSpeed =  80,.minSpeed = 10});
    chassis.waitUntilDone();
    chassis.moveToPoint(-22, 50,2000, {.forwards = true,.maxSpeed =  80,.minSpeed = 10});
    chassis.waitUntilDone();
    pros::delay(2000);
    stopIntake();
    chassis.waitUntilDone();
    chassis.turnToPoint(15, 42, 1500);
     chassis.moveToPoint(15, 42, 3000, {.forwards = true,.maxSpeed = 70,.minSpeed = 10});


}
void redRight()// Works
{
        chassis.setPose(0,0,180);
        chassis.moveToPoint(0,  32, 2000, {.forwards = false,.maxSpeed = 70,.minSpeed = 10});
        chassis.waitUntilDone();
        pros::delay(600);
        piston.set_value(true);
        pros::delay(500);
        moveIntakeforward();
        chassis.turnToPoint(32, 37, 1000);
        chassis.moveToPoint(32, 37, 3000);
        chassis.turnToPoint(-32, 37, 1500);
        chassis.moveToPoint(-45, 37, 3000, {.forwards = true,.maxSpeed = 80,.minSpeed = 10});
        pros::delay(2000);
        stopIntake();

}

void blueLeft() //Doesn't work //19.96,-36.3
{
    chassis.setPose(0,0,180);
        chassis.moveToPoint(0,  32, 2000, {.forwards = false,.maxSpeed = 70,.minSpeed = 10});
        chassis.waitUntilDone();
        pros::delay(600);
        piston.set_value(true);
        pros::delay(500);
        moveIntakeforward();
        chassis.turnToPoint(-32, 37, 1000);
        chassis.moveToPoint(-32, 37, 3000);
        chassis.turnToPoint(32, 37, 1500);
        pros::delay(1000);
        stopIntake();
        chassis.moveToPoint(32, 37, 3000, {.forwards = true,.maxSpeed = 80,.minSpeed = 10});
}
    

    /*
    chassis.turnToPoint(-20,50,1000)
    */

void blueRight() { // Works
    chassis.setPose(0,0,180);
    chassis.moveToPoint(0,  32, 2000, {.forwards = false,.maxSpeed = 70,.minSpeed = 10});
    chassis.waitUntilDone();        
    pros::delay(600);
    piston.set_value(true);
    pros::delay(1500);
    moveIntakeforward();
    chassis.turnToPoint(32, 37, 1000);
    chassis.moveToPoint(32, 37, 3000);
    chassis.turnToPoint(-32, 37, 1500);
    chassis.moveToPoint(-64, 37, 3000, {.forwards = true,.maxSpeed = 80,.minSpeed = 10});
}

void testblueRight()
{
    chassis.setPose(0,0,180); // 
    chassis.moveToPoint(0,  35, 2000, {.forwards = false,.maxSpeed = 70,.minSpeed = 10});
    chassis.waitUntilDone();        
    pros::delay(600);
    piston.set_value(true);
    pros::delay(1500);
    moveIntakeforward();
    chassis.turnToPoint(31, 35, 1000);
    chassis.moveToPoint(31, 35, 3000);
    chassis.turnToPoint(31, 49, 1000);
    chassis.moveToPoint(31, 49,2000,{.forwards = true,.maxSpeed =  90,.minSpeed = 10});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(31, 37,2000,{.forwards = false,.maxSpeed =  80,.minSpeed = 10});
    chassis.waitUntilDone();
    chassis.moveToPoint(27, 50,2000, {.forwards = true,.maxSpeed =  80,.minSpeed = 10});
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.turnToPoint(-30, 37, 1500);
    stopIntake();
    chassis.waitUntilDone();
    chassis.moveToPoint(-30, 37, 3000, {.forwards = true,.maxSpeed = 70,.minSpeed = 10});

}
void testredleft() {
    chassis.setPose(0,0,180);
    chassis.moveToPoint(0,  40, 2000, {.forwards = false,.maxSpeed = 70,.minSpeed = 10});
    chassis.waitUntilDone();        
    pros::delay(600);
    piston.set_value(true);
    pros::delay(1500);
    moveIntakeforward();
    chassis.turnToPoint(-29, 35, 1000);
    chassis.moveToPoint(-29, 35, 3000);
    chassis.turnToPoint(-31, 47, 1000);
    chassis.moveToPoint(-31, 47,2000,{.forwards = true,.maxSpeed =  90,.minSpeed = 10});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(-31, 37,2000,{.forwards = false,.maxSpeed =  80,.minSpeed = 10});
    chassis.waitUntilDone();
    chassis.moveToPoint(-22, 50,2000, {.forwards = true,.maxSpeed =  80,.minSpeed = 10});
    chassis.waitUntilDone();
    pros::delay(2000);
    stopIntake();
    chassis.waitUntilDone();
    chassis.turnToPoint(15, 42, 1500);
     chassis.moveToPoint(15, 42, 3000, {.forwards = true,.maxSpeed = 70,.minSpeed = 10});

}


void goldRushRed()
{
    //Clamps 5th goal using Doinker
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 50, 2000); //Moves to Doinker
    pros::delay(1000);//Delay so that doinker can clamp on the mogo at the right time
    doinker.set_value(true); //setting doinker to clamp the mogo
    
    //Moves Back and clamps the fifth goal with mogo && scored preLoad
    chassis.moveToPoint(0,30,2000, {.forwards = false,.maxSpeed = 70,.minSpeed = 10}); //Moving Back with the doinker 
    chassis.waitUntilDone();//So that it doesn't set the doinker to false instantly
    doinker.set_value(false);//un -clamping the mogo
    chassis.turnToHeading(160, 1500);

    chassis.moveToPoint(-7, 43, 2000,{.forwards = false,.maxSpeed = 70,.minSpeed = 10});//Moving backwards with the mogo mech facing the 5th goal
    chassis.waitUntilDone();//waiting until movement is completed
    piston.set_value(true);//Setting mogo mech to clamp mode
    pros::delay(1000);
    moveIntakeforward();// Moves the intake 
    pros::delay(1000);
    piston.set_value(false);//Unclamping the 5th mogo
    // pros::delay(1000);
    // stopIntake();

   
    
    

}

void autonomous() 
{
    goldRushRed();

}
 
 
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

    bool pistonState = false;
    bool doinkerState = true;

	
    
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
       // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 0.75;
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.75;
		int lowerIntakePower = 127;
        int upperIntakePower = lowerIntakePower ;


        chassis.curvature(leftY, rightX);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) 
        {
            pistonState = !pistonState;
            piston.set_value(pistonState);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        {
            doinkerState = !doinkerState;
            doinker.set_value(doinkerState);
        }

        

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_lower.move(lowerIntakePower);   
            intake_upper.move(upperIntakePower); 
         }
         else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) 
        {
            intake_lower.move(-1*lowerIntakePower);   
            intake_upper.move(-1*upperIntakePower); 
        }
        else
        {
            intake_lower.move(0);   
            intake_upper.move(0); 
        }


        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) 
        {
			nextState();
		}

		// if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) 
        // {
        //     arm.move_absolute(-60 , 125);
            

		// } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) 
        // {
		// 	arm.move_absolute(0, 125);

		// } 

        // if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) 
        // {

        //     arm.move_absolute(-300, 125);

		// }if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        //     arm.move_absolute(arm.get_position(), 120);
        // }

        pros::delay(20);
	
	}
}
