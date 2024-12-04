#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

double trackWidth = 11.25;

pros::MotorGroup left_motors({-20, -13, -11}, pros::MotorGearset::blue); // left motors on ports NEED ACTUAL PORTS
pros::MotorGroup right_motors({19, 14, 12}, pros::MotorGearset::blue); // left motors on ports NEED ACTUAL PORTS

lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              trackWidth, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Motor intake(17, pros::MotorGearset::blue);

pros::Motor lift(18, pros::MotorGearset::red);

pros::Distance dist(15);

pros::adi::DigitalOut mogo('A');

pros::adi::DigitalOut cclear('C');

pros::adi::DigitalOut rLock('B');

pros::Imu imu(16);
// Toggle and Latch for Mobile Goal Mech
bool mToggle = false;

bool mLatch = false;
// Toggle and Latch for Corner Clear
bool cLatch = false;

bool cToggle = false;
// Toggle and Latch for ring lock
bool lLatch = false;

bool lToggle = false;
//
// lateral PID controller
lemlib::ControllerSettings lateral_controller(213, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              70, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
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
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
						sensors,
						&throttle_curve,
						&steer_curve
);

void ringDetected(){
    while(true){
        if((dist.get_distance() <= 2.5) && lLatch == false){
            pros::delay(500);
            lLatch = true;
            rLock.set_value(true);
        } else if ((dist.get_distance() > 2.5) && lLatch == true){
            lLatch = false;
            rLock.set_value(false);
        }
    }
}




void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

ASSET(pathMogoAfter_txt);

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Heading|Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    //lateral test
    //chassis.setPose(0, 0, 90);
    //chassis.moveToPoint(48, 0, 100000);
    //angular test
    //chassis.turnToHeading(90, 1000000);

    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, 90);
    intake.move(114);
    lift.move(-30);
    pros::delay(1000);
    lift.brake();

    chassis.moveToPose(-46, 0, 90, 3000);
    void update();
    chassis.turnToHeading(0, 3000);
    intake.move(-114);
    chassis.moveToPoint(-49, -23.047, 3000, {.forwards = false, .maxSpeed = 70});
    void update();
    pros::delay(500);
    mogo.set_value(true);
    
    chassis.turnToHeading(90, 2000);
    void update();
    intake.move(114);
    chassis.moveToPoint(-30, -12, 1500, {.maxSpeed = 80});
    void update();
    chassis.turnToHeading(110, 2000);
    void update();
    chassis.moveToPoint(0, -25, 3000, {.maxSpeed = 70});
    void update();
    chassis.turnToHeading(180, 2000);
    void update();
    chassis.moveToPoint(0, -38, 3000, {.maxSpeed = 70});
    void update();
    pros::delay(3000);
    chassis.moveToPoint(-5, -30, 4000, {.forwards = false, .maxSpeed = 60});
    void update();
    pros::delay(2000);
    chassis.turnToHeading(270, 2000);
    void update();
    pros::delay(1000);
    chassis.moveToPoint(-50, -37  , 5000, {.maxSpeed = 50});
    void update();
    pros::delay(4000);
    chassis.moveToPoint(-20, -37, 5000, {.forwards = false, .maxSpeed = 60});
    void update();
    chassis.moveToPoint(-42, -50, 2000, {.maxSpeed = 40});
    void update();
    pros::delay(3000);
    chassis.turnToHeading(60, 2000);
    
    chassis.moveToPoint(-55, -55, 2000, {.forwards = false, .maxSpeed = 40});
    void update();
    pros::delay(4000);
    mogo.set_value(false);
    void update();
    chassis.moveToPoint(-54, -45, 2000, {.maxSpeed = 40});
    void update();
    chassis.turnToHeading(180, 2000);
    chassis.moveToPoint(-45, 23, 4000,{.forwards = false, .maxSpeed = 80});
    void update();
    
    
    //intake.move(114);
    //chassis.moveToPose(-60, 0, 270, 3000);
    
    //chassis.turnToHeading(90, 10000000000);
}// -49.827x -19.89y 9.8 deg
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

 //Wy Leekai Yoti Wiley Coyote
pros::Controller controller(pros::E_CONTROLLER_MASTER);
void opcontrol() {
    
	while (true) {
		
        ///////////////Drive Controls///////////////////
        
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(leftY, rightX);

        ////////////////////////////////////////////////

        //////////////Intake Controls///////////////////

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move(127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move(-127);
        } else {
            intake.move(0);
        }

        ////////////////////////////////////////////////

        //////////////Lift Controls/////////////////////

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            lift.move(-127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            lift.move(127);
        }   else {
            lift.move(0);
        } 

        ////////////////////////////////////////////////

        ////////////Mobile Goal Mech////////////////////

        mogo.set_value(mToggle);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            if(!mLatch){
                mToggle = !mToggle;
                mLatch = true;
            }
        } else {
            mLatch = false;
        }

        ////////Corner Clear////////////////////////////

        if (cToggle){
            cclear.set_value(true);
        } else {
            cclear.set_value(false);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            if(!cLatch){
                cToggle = !cToggle;
                cLatch = true;
            }
        } else {
            cLatch = false;
        }


		pros::delay(20);                               // Run for 20 ms then update
	}
}