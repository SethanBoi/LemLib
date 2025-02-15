#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "backpack.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// left motor group
pros::MotorGroup left_motor_group({-1, -3, -5}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({2, 4, 6}, pros::MotorGears::blue);


pros::Motor Stage2(7, pros::MotorGearset::green);  // Intake motor
pros::Motor Stage1(18, pros::MotorGearset::green);
pros::Motor Lift(8, pros::MotorGearset::green); // Lift motor

// Digital Outputs
pros::adi::DigitalOut Mogo('A');     
pros::adi::DigitalOut Arm('B');     
pros::adi::DigitalOut Yoinker('H');  

pros::Rotation LiftSensor(9);   

// Inertial Sensor on Port 140

pros::Rotation vertical_encoder(10);   
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);

pros::Imu imu(14);
team_e::Backpack backpack{};

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);



// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
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
                                              15 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                           	45, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);


bool clamp = false;
bool armdown = false;
bool yoink = false;
bool isintaking = false;
int liftingStage = 0;
int lifting = 0;
void initialize() {
	imu.reset();
    controller.clear();
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });	

	//armlift
	pros::Task armLift([&]() {
		while(true){
        	double liftrot = LiftSensor.get_position() / 100.0;
        	pros::delay(100); 

        	if (lifting == 1 && liftingStage == 0) {
            	Lift.move_velocity(30);
            	liftrot = LiftSensor.get_position() / 100.0;
            	if (liftrot > 3) {
            	    Lift.move_velocity(0);
            	    Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            	    lifting = 0;
            	}
        	}
        	else if (lifting == 1 && liftingStage == 1) {
            	Lift.move_velocity(120);
            	liftrot = LiftSensor.get_position() / 100.0;
            	if (liftrot > 90) {
            	    Lift.move_velocity(0);
            	    Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
             	   	lifting = 0;
            	}
        	}
        	else if (lifting == 1 && liftingStage == 2) {
            	Lift.move_velocity(-100);
            	liftrot = LiftSensor.get_position() / 100.0;
            	if (liftrot < 2) {
               	 	Lift.move_velocity(0);
               	 	Lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                	lifting = 0;
            	}
        	}
			pros::delay(10);
		}
    });	
	controller.print(0, 0, "Calibrating Inertial Sensor...");
    while (imu.is_calibrating()) {
        pros::delay(100);
    }
    controller.clear();
    controller.print(0, 0, "Inertial Calibrated");


}
/*
std::string autonNames[] = {"Disabled", "Auton 1", "Auton 2"};
int selectedAuton = 0;
int totalAutons = sizeof(autonNames) / sizeof(autonNames[0]);

void displayAutonSelector() {
    while (true) {
        pros::lcd::clear();
        pros::lcd::print(0, "Autonomous Selector");
        pros::lcd::print(1, "Selected: %s", autonNames[selectedAuton].c_str());
        pros::delay(100);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            selectedAuton = (selectedAuton - 1 + totalAutons) % totalAutons;
            pros::delay(300); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            selectedAuton = (selectedAuton + 1) % totalAutons;
            pros::delay(300); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            break; 
        }
    }
    pros::lcd::clear();
    pros::lcd::print(0, "Selected: %s", autonNames[selectedAuton].c_str());
}
*/



//AUTON SELECTOR
int selectedAuton = 1;  
int numAutons = 1;
std::string team = "red";
void auton_selector() {
    std::string name;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {

        selectedAuton = selectedAuton + 1;
        if (selectedAuton > numAutons) {
            selectedAuton = 1;
        }

        pros::delay(500);

    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {

        selectedAuton = selectedAuton - 1;
        if (selectedAuton <= 0) {
            selectedAuton = numAutons;
        }

        pros::delay(500);

    }

    if (selectedAuton == 1) {
        name = "skills";
        team = "red";
    }
    controller.print(0, 0, "Auton: %d %s", selectedAuton, name);
}




void auto1(){
		LiftSensor.reset_position();
		backpack.move(90.0, 200);
		chassis.setPose(-60, 0, -90);
		chassis.moveToPoint(-43, 0, 2000, {.forwards = false});
		backpack.move(0, 160);
		chassis.turnToPoint(-43, -18.5, 2000, {.forwards = false});
    	chassis.moveToPoint(-43, -18.5, 2000, {.forwards = false,.maxSpeed = 50});
		pros::delay(1000); //
		Mogo.set_value(true);
		pros::delay(300);
		Stage1.move_velocity(200);
		Stage2.move_velocity(200);
		//first corner
    	chassis.turnToPoint(-22.5, -23.5, 2000, {.maxSpeed = 90});
		Stage1.move_velocity(200);
		Stage2.move_velocity(200);
    	chassis.moveToPoint(-22.5, -23.5, 2000, {.maxSpeed = 80});
		chassis.turnToPoint(-22.5, -47, 2000);
		chassis.moveToPoint(-22.5, -47, 2000);
		chassis.turnToPoint(-55.5, -47, 2000);
		//chassis.moveToPoint(-46, -47, 2000);
		//pros::delay(300);
		//Stage1.move_velocity(200);
		//Stage2.move_velocity(200);
		chassis.moveToPoint(-55.5, -47, 2000,{.maxSpeed = 60});
		chassis.turnToPoint(-48.8, -57.1, 2000);
		chassis.moveToPoint(-48.8, -57.1, 2000);
		pros::delay(2450); 
		Stage1.move_velocity(0);
		Stage2.move_velocity(0);
		chassis.turnToPoint(-47, -47, 2000);
		chassis.moveToPoint(-57, -57, 2000, {.forwards = false});
		Mogo.set_value(false);
		//1sttransition
		chassis.moveToPoint(-45, -47, 3000);
		chassis.turnToPoint(-45, 13, 3000, {.forwards = false});
		pros::delay(400);
    	chassis.moveToPoint(-45, 13, 3000, {.forwards = false, .maxSpeed = 77});
		//chassis.moveToPoint(-47, 19.5, 2000, {.forwards = false, .maxSpeed = 40});
		pros::delay(1500);
		Mogo.set_value(true);
		//2nd corner
		chassis.turnToPoint(-22.5, 23.5, 2000, {.maxSpeed = 80});
		Stage1.move_velocity(200);
		Stage2.move_velocity(200);
    	chassis.moveToPoint(-23.5, 23.5, 2000, {.maxSpeed = 80});
		chassis.turnToPoint(-21, 47, 2000);
		chassis.moveToPoint(-21, 47, 2000);
		chassis.turnToPoint(-54.5, 47, 2000);
		//chassis.moveToPoint(-46, 47, 2000);
		//pros::delay(300);
		//Stage1.move_velocity(200);
		Stage2.move_velocity(200);
		chassis.moveToPoint(-55.5, 47, 5000,{.maxSpeed = 70});
		chassis.turnToPoint(-48.8, 57.1, 5000);
		chassis.moveToPoint(-48.8, 57.1, 5000);
		pros::delay(2300); 
		Stage1.move_velocity(0);
		Stage2.move_velocity(0);
		chassis.turnToPoint(-47, 47, 5000);
		chassis.moveToPoint(-57, 57, 5000, {.forwards = false});
		Mogo.set_value(false);
		//transition 
		chassis.moveToPoint(-47, 54, 5000, {.maxSpeed = 100});
		chassis.turnToPoint(30, 51, 4000);
		//chassis.moveToPoint(-72, 47, 2000, {.forwards = false});
		Stage1.move_velocity(200);
		Stage2.move_velocity(200);
		chassis.moveToPoint(30, 51, 5000, {.maxSpeed = 80});
		pros::delay(1940);
		Stage1.move_velocity(0);
		Stage2.move_velocity(0);
		//chassis.turnToPoint(45.3, 11.8, 4000, {.forwards = false});
		//chassis.moveToPoint(45.3, 11.8, 4000, {.forwards = false,.maxSpeed = 75});
		//chassis.turnToPoint(51.5, 2.5, 4000, {.forwards = false});
		//chassis.moveToPoint(51.5, 2.5, 4000, {.forwards = false,.maxSpeed = 73});
		chassis.turnToPoint(56.5, 1, 4000, {.forwards = false});
		chassis.moveToPoint(56.5, 1, 5000, {.forwards = false, .maxSpeed = 80});
		pros::delay(1450);
		Mogo.set_value(true);
		//3rd mogo 
		Stage1.move_velocity(200);
		Stage2.move_velocity(200);
		//chassis.turnToPoint(23.5, 23.5, 2000);
		//chassis.moveToPoint(23.5, 23.5, 2000);
		//chassis.turnToPoint(0, 0, 2000);
		//chassis.moveToPoint(0, 0, 2000);
		chassis.turnToPoint(25, -25, 2000);
		chassis.moveToPoint(25, -25, 2000);
		chassis.turnToPoint(25, -47, 2000);
		chassis.moveToPoint(25, -47, 2000);
		chassis.turnToPoint(47.6, -49, 2000);
		chassis.moveToPoint(47.6, -49, 2000);
		chassis.turnToPoint(65, -64, 2400, {.forwards = false});
		chassis.moveToPoint(65, -64, 2400, {.forwards = false});
		pros::delay(600);
		Mogo.set_value(false);
		//MOGO 4
		chassis.moveToPoint(48, -24, 5000);
		chassis.turnToPoint(85, 58, 5000);
		chassis.moveToPoint(85, 58, 5000);
		//han
		


}


//auton picker
void autonomous() {
    switch (selectedAuton) {
        case 1:
			auto1();
            break;
}

}
void handle_intake(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !isintaking) {
        Stage1.move_velocity(200);
		Stage2.move_velocity(200);
        isintaking = true;
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !isintaking) {
    	Stage1.move_velocity(-150);
		Stage2.move_velocity(-150);
        isintaking = true;
    }
    else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
             !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && isintaking) {
        Stage1.move_velocity(0);
Stage2.move_velocity(0);
        isintaking = false;
    }
}

void handle_toggles(){
    // Toggle Mogo Button A
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        clamp = !clamp;
        Mogo.set_value(clamp);
    }

    // Toggle Yoinker Button X
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        yoink = !yoink;
        Yoinker.set_value(yoink);
    }

    // Toggle Arm Button B
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        armdown = !armdown;
        Arm.set_value(!armdown);
    }

	
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        auto1();
    }

}

bool button_pressed = false;
void arm_control(){
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
		if (!button_pressed) {
			lifting = 1;
			liftingStage = (liftingStage + 1) % 3;
			button_pressed = true;
		}
	}
	else{
		lifting = 0;
		button_pressed = false;
	}
}

void opcontrol() {
    // loop forever
    while (true) {
		
        // get left y and right y positions
		auton_selector();
		handle_intake();
        handle_toggles();
		arm_control();

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
} 