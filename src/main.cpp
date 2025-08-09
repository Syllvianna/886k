#include "main.h"
Controller master(pros::E_CONTROLLER_MASTER);
MotorGroup left_mg({-11, -12});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
Motor leftFrontMotor(-11);
Motor leftBackMotor(-12);
MotorGroup right_mg({19, 20});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
Motor rightFrontMotor(19);
Motor rightBackMotor(20);
MotorGroup fourbar_Arm_mg({-1, 10});
Motor leftfourbar(10);
Motor rightfourbar(-1);
Motor intake({21});
pros::adi::Pneumatics left_piston('a', false);
pros::Imu imu_sensor(13);

void powerDrive(int power, int turn){
	left_mg.move(power + turn);
	right_mg.move(power - turn);
}

void driveForward(int distance, double kP){
	left_mg.tare_position();
	int error = distance - left_mg.get_position();
	while(abs(error) > 2) {
		error = distance - left_mg.get_position();
		powerDrive(error * kP, 0);
	}
	powerDrive(0, 0);
}

void turn(int degrees, double kP){
	imu_sensor.tare_rotation();
	int error = degrees - imu_sensor.get_rotation();
	while(error > 0) {
		error = degrees - imu_sensor.get_rotation();
		powerDrive(0, -error * kP);
	}
	powerDrive(0, 0);
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu_sensor.reset();

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

	turn(90, 0.4);
	/*
	driveForward(1000, 0.4);
	turn(45, 0.4);
	intake.move(-127);
	driveForward(700, 0.4);
	intake.move(0);
	driveForward(300,0.4);
	intake.move(127);
	delay(0.5);
	turn(45, 0.4);
	driveForward(500, 0.4);
	intake.move(127);
	driveForward(500, 0.4);
	intake.move(0);
	turn(-60, 0.4);
	driveForward(300, 0.4);
	intake.move(-127);
	intake.move(0);
	driveForward(-1000, 0.4);
	turn(-50, 0.4);
	left_piston.extend();
	intake.move(127);
	driveForward(500, 0.4);
	driveForward(-100, 0.4);
	driveForward(100, 0.4);
	driveForward(-300, 0.4);
	intake.move(0);
	left_piston.retract();
	turn(-180, 0.4);
	fourbar_Arm_mg.move_absolute(-1500, 100);
	driveForward(300, 0.4);
	intake.move(-127);
	delay(2);
	intake.move(0);
	driveForward(-300, 0.4);
	turn(90, 0.4);
	driveForward(100, 0.4);
	turn(90, 0.4);
	driveForward(3000, 0.4);


	
	fourbar_Arm_mg.move_absolute(-1500 , 100);
	driveForward(1000, 0.4);
	fourbar_Arm_mg.move(-50);
	intake.move(127);
	*/
}

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
void opcontrol() {


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
		left_piston.extend();
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
		left_piston.retract();
		}



		if (master.get_digital(DIGITAL_L1)==1){
			fourbar_Arm_mg.move(-100); 
		} else if (master.get_digital(DIGITAL_L2)==1){
			fourbar_Arm_mg.move(100); 
		} else {
			fourbar_Arm_mg.move(0);
		}

		if (master.get_digital(DIGITAL_R1)==1){
			intake.move(-127); 
		} else if (master.get_digital(DIGITAL_R2)==1){
			intake.move(127); 
		} else {
			intake.move(0);
		}

		pros::delay(20);               // Run for 20 ms then update
		
		
	}
}