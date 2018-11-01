/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Drive/DifferentialDrive.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <XboxController.h>
#include <Spark.h>
#include <VictorSP.h>
#include <SpeedControllerGroup.h>
#include "WPILib.h"
// #include <NidecBrushless.h>
#include <DoubleSolenoid.h>
#include <Timer.h>
#include <Encoder.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot: public frc::IterativeRobot {

	// Here we turn every motor on a pin into an object
	// Then turn the left/right groups into a SpeedControllerGroup class
	// And hand the groups over to DifferentialController instead of each motor individually.

	//

	// no touchy frc::DoubleSolenoid dblSolenoid{0,1};

	frc::ADXRS450_Gyro gyro;

	frc::Encoder encoder_left { 9, 8, false, Encoder::EncodingType::k4X };

	frc::Timer m_timer;

	frc::VictorSP m_arm_left { 6 };
	frc::VictorSP m_arm_right { 7 };

	frc::VictorSP m_elevator_1 { 5 };
	frc::VictorSP m_elevator_2 { 4 };

	frc::VictorSP m_leftFrontMotor { 0 };
	frc::VictorSP m_leftRearMotor { 1 };

	frc::SpeedControllerGroup m_left { m_leftFrontMotor, m_leftRearMotor };

	frc::VictorSP m_rightFrontMotor { 2 };
	frc::VictorSP m_rightRearMotor { 3 };

	frc::SpeedControllerGroup m_right { m_rightFrontMotor, m_rightRearMotor };

	frc::DifferentialDrive m_robotDrive { m_left, m_right };

	frc::Joystick m_stick { 0 };

	// Initiating the XBox controller(s)

	frc::XboxController driver_controller { 0 };
	frc::XboxController button_controller { 1 };

	frc::AnalogInput ultra_1 { 0 };

public:

	void RobotInit() {
		//Preferences from SmartDashboard
		// TODO: Make it work.
		//Preferences from SmartDashboard
		// TODO: Make it work.

		m_timer.Start();
		CameraServer::GetInstance()->StartAutomaticCapture();
		prefs = Preferences::GetInstance();
		pneumatics_enable = prefs->GetBoolean("PneumaticsEnable", false);
		if (pneumatics_enable) {
			dblSolenoid = new DoubleSolenoid(0, 1);
		}

		encoder_left.Reset();

	}

	void DisabledInit() {
		m_robotDrive.ArcadeDrive(0.0, 0.0);
	}

	void DisabledPeriodic() {
		drive_multiplier = prefs->GetDouble("DriveMultiplier", 0.75);
		turn_multiplier = prefs->GetDouble("TurnMultiplier", 0.70);
		climber_multiplier = prefs->GetDouble("ClimberMultiplier", 1);
		snap_distance_top = prefs->GetDouble("SnapDistanceTop", 0.05);
		snap_distance_bottom = prefs->GetDouble("SnapDistanceBottom", 0.2);
		ramp_multiplier = prefs->GetDouble("RampMultiplier", 0.03);

		auto_time = prefs->GetInt("AutoTime", 1);
		auto_amount = prefs->GetDouble("AutoAmount", 0.01);
		box_motor = prefs->GetDouble("BoxMotor", 0.5);
		gyrokp = prefs->GetFloat("gyrokp", 0.03);
		gyro_enable = prefs->GetBoolean("GyroEnable", false);
		climber_hold = prefs->GetDouble("ClimberHold", 0.10);
		mid_auto = prefs->GetBoolean("MidAuto", true);

		// Show preferences in SmartDashboard (mainly for debugging)
		SmartDashboard::PutNumber("Drive Multiplier:", drive_multiplier);
		SmartDashboard::PutNumber("Turn Multiplier:", turn_multiplier);
		SmartDashboard::PutNumber("Climber Multiplier:", climber_multiplier);
		SmartDashboard::PutNumber("Snap Distance Top:", snap_distance_top);
		SmartDashboard::PutNumber("Snap Distance Bottom:",
				snap_distance_bottom);
		SmartDashboard::PutNumber("Ramp Multiplier:", ramp_multiplier);
		SmartDashboard::PutBoolean("Pneumatics Enabled:", pneumatics_enable);
		SmartDashboard::PutNumber("Auto Time:", auto_time);
		SmartDashboard::PutNumber("Auto Amount:", auto_amount);
		SmartDashboard::PutNumber("Box Motor Multiplier:", box_motor);
		SmartDashboard::PutNumber("Gyro kP:", gyrokp);
		SmartDashboard::PutNumber("Gyro Enable:", gyro_enable);
		SmartDashboard::PutNumber("Climber Hold:", climber_hold);
		SmartDashboard::PutBoolean("Middle Auto:", mid_auto);
		m_robotDrive.ArcadeDrive(0.0, 0.0);
		m_elevator_1.Set(-climber_hold);
		Wait(1);
	}
	// :)

	float getWaitTime(int stage) {
		if (stage == 0) {
			return stageLength[0];
		}
		float addingTime;
		for (int i = 0; i < stage + 1; i++) {
			addingTime += stageLength[i];
		}
		return addingTime;
	}

	void AutonomousInit() {
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData[0] == 'L') {
				leftRightNegative = -1;
			} else {
				leftRightNegative = 1;
			}
		}
		gyro.Reset();
		m_timer.Reset();
		m_timer.Start();

		if (pneumatics_enable) {
			dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
		}

		if (mid_auto) {
			stageLength[0] = 2;
			stageLength[1] = 1;
			stageLength[2] = 0.1;
			stageLength[3] = 2;
			stageLength[4] = 0.2;
			stageLength[5] = 1.4;
			stageLength[6] = 2;
			stageLength[7] = 0.1;
			stageLength[8] = 1.5;
			stageLength[9] = 1;
		} else {
			stageLength[0] = 6;
		}
	}

	void AutonomousPeriodic() {
		m_elevator_1.Set(climber_hold);
		float angle = -gyro.GetAngle(); // get heading
		SmartDashboard::PutNumber("Gyro Angle:", angle);
		float targetHeading;
		float error;
		//SmartDashboard::PutNumber("Gyro Angle:", angle);
		float time = m_timer.Get();
		if (mid_auto) {
			if (time < getWaitTime(0)) {
				m_elevator_1.Set(-0.75);
			} else if (time < getWaitTime(1)) {
				SmartDashboard::PutNumber("Auto Stage:", 0);
				m_robotDrive.ArcadeDrive(0.7, -angle * gyrokp); // turn to correct heading

				Wait(0.004);

			} else if (time < getWaitTime(2)) {
				m_elevator_1.Set(-climber_hold);
				SmartDashboard::PutNumber("Auto Stage:", 1);
				gyro.Reset();
			} else if (time < getWaitTime(3)) {
				targetHeading = leftRightNegative * 90;
				error = angle - targetHeading;
				angle = -gyro.GetAngle();
				if (abs(error) < 35) {
					SmartDashboard::PutNumber("Auto Stage:", 2.5);
					m_robotDrive.ArcadeDrive(0.0,
							leftRightNegative * 0.075 * -gyrokp * (error));
					SmartDashboard::PutNumber("Gyro Kp result:",
							(0.26 * -gyrokp * (error)));
				} else {
					SmartDashboard::PutNumber("Auto Stage:", 2);
					m_robotDrive.ArcadeDrive(0.0, leftRightNegative * 0.5);
				}
			} else if (time < getWaitTime(4)) {
				SmartDashboard::PutNumber("Auto Stage:", 3);
				gyro.Reset();
			} else if (time < getWaitTime(5)) {
				SmartDashboard::PutNumber("Auto Stage:", 4);
				m_robotDrive.ArcadeDrive(0.7, -angle * gyrokp); // turn to correct heading
				Wait(0.004);
			} else if (time < getWaitTime(6)) {

				targetHeading = leftRightNegative * -90;
				error = angle - targetHeading;
				angle = -gyro.GetAngle();
				if (abs(error) < 35) {
					SmartDashboard::PutNumber("Auto Stage:", 5.5);
					m_robotDrive.ArcadeDrive(0.0,
							leftRightNegative * -0.075 * gyrokp * (error));
					SmartDashboard::PutNumber("Gyro Kp result:",
							(0.26 * -gyrokp * (error)));
				} else {
					SmartDashboard::PutNumber("Auto Stage:", 5);
					m_robotDrive.ArcadeDrive(0.0, leftRightNegative * -0.5);
				}
			} else if (time < getWaitTime(7)) {
				SmartDashboard::PutNumber("Auto Stage:", 6);
				gyro.Reset();
			} else if (time < getWaitTime(8)) {
				SmartDashboard::PutNumber("Auto Stage:", 4);
				m_robotDrive.ArcadeDrive(0.7, -angle * gyrokp); // turn to correct heading

				Wait(0.004);
			} else if (time < getWaitTime(9)) {
				SmartDashboard::PutNumber("Auto Stage:", 6);
				m_arm_left.Set(0.6);
				m_arm_right.Set(-0.6);
			} else {
				m_arm_left.Set(0);
				m_arm_right.Set(0);
			}
		} else {
			if (time < getWaitTime(0)) {
				SmartDashboard::PutNumber("Auto Stage:", 0);
				m_robotDrive.ArcadeDrive(0.55, -angle * gyrokp); // turn to correct heading
				SmartDashboard::PutNumber("Gyro Kp result:", -angle * gyrokp);
				Wait(0.004);

			}
		}

		//SmartDashboard::PutNumber("Gyro Angle:", angle);

	}

	void TeleopInit() {
		if (pneumatics_enable) {
			dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
		}
	}

	void TeleopPeriodic() {
		getInput();

		if (button_y_2) {
			if (force_slow > 0 && force_slow < 0.4) {
				force_slow -= 0.1;
			}
		} else {
			if (force_slow < 1) {
				force_slow += 0.1;
			}
		}

		// drive with arcade style
		//Make current value get closer to target
		target = (input_lt - input_rt) * drive_multiplier * force_slow;
		if (target - current < snap_distance_top
				&& target - current > -snap_distance_top)
			current = target;
		if (input_lt == 0.0 && input_rt == 0.0) { // When there's no input, stop the bus
			current += (target - current) * (ramp_multiplier + 0.1);
		} else {
			current += (target - current) * ramp_multiplier;

			// If in this limbo area, snap out of it!
			if (current < 0 && current > -snap_distance_bottom) {
				if (input_rt != 0.0) {    // Right Trigger held, pull up
					current = -snap_distance_bottom;
				} else {    // Right trigger isn't held, snap down to zero
					current = 0;
				}
			} else if (current > 0 && current < snap_distance_bottom) {
				if (input_lt != 0.0) {    // Right Trigger held, pull up
					current = snap_distance_bottom;
				} else {    // Right trigger isn't held, snap down to zero
					current = 0;
				}
			}

		}

		if (button_x_prev != button_x && button_x) {
			holdClimber = !holdClimber;
		}

		if (button_rb) {
			m_elevator_1.Set(-climber_multiplier);
			holdClimber = false;
		} else if (button_lb) {
			m_elevator_1.Set(climber_multiplier / 4);
			holdClimber = false;
		} else if (holdClimber) {
			m_elevator_1.Set(0);
		} else {
			m_elevator_1.Set(climber_hold);
			holdClimber = false;
		}

		button_x_prev = button_x;

		m_robotDrive.ArcadeDrive(-current,
				input_left_stick_x * turn_multiplier);

		// Put on dashboardy
		SmartDashboard::PutNumber("Target Speed:", -target);
		SmartDashboard::PutNumber("Current Speed:", -current);
		SmartDashboard::PutNumber("Current Stick:", input_left_stick_x);
		SmartDashboard::PutNumber("Current Turn:",
				input_left_stick_x * (-target));
		SmartDashboard::PutNumber("Ultrasonic 1:",
				(ultra_1.GetVoltage() / 0.001) / 45);
		count_left = encoder_left.Get();
		SmartDashboard::PutNumber("Counter Left", count_left);
		SmartDashboard::PutNumber("Force Slow", force_slow);


		if (pneumatics_enable) {
			if (button_a) {
				dblSolenoid->Set(DoubleSolenoid::Value::kForward);
			}
			if (button_b) {
				dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
			}
		}

		if (input_b_rt != 0) {
			m_arm_left.Set(box_motor * -input_b_rt);
			m_arm_right.Set(-box_motor * -input_b_rt);
		} else if (input_b_lt != 0) {
			m_arm_left.Set(-box_motor * -input_b_lt);
			m_arm_right.Set(box_motor * -input_b_lt);
		} else {
			m_arm_left.Set(0.0);
			m_arm_right.Set(0.0);
			//yo how ya doin boi
		}

	}

private:

	bool pneumatics_enable;
//Robot Preferences from SmartDashboard
	Preferences *prefs;
//Value from 0.0 to 1.0, reduces motor speed to prevent issues during testing.
	double drive_multiplier; // Default 0.75
	double snap_distance_top; //Default is 0.05
	double snap_distance_bottom; //Default is 0.2
	double ramp_multiplier; //Default is 0.03
	double turn_multiplier; // Default 0.50
	double climber_multiplier; // Default 0.3
	double box_motor;
	double climber_hold;

	int time, time_2 = 0;

//Autonomous Values
	double auto_time;
	double auto_amount;

	float gyrokp;
	int leftRightNegative = 0;
	float stageLength[10];
	bool mid_auto;

//Pneumatics
	DoubleSolenoid *dblSolenoid;

//Drive Values
	double current = 0.0, target = 0.0;

	float force_slow = 1.0;

//Analog stick input values
	double input_left_stick_y, input_right_stick_y, input_left_stick_x,
			input_right_stick_x;

//Trigger Values
	double input_lt, input_rt, input_b_lt, input_b_rt;

//Face button values
	bool button_a, button_b, button_x, button_y, button_y_2;

	bool button_x_prev, holdClimber, ignoreNext = false;

//Bumpers

	bool button_lb, button_rb;

	bool gyro_enable;

	int count_left;

//Encoder *sampleEncoder;

	void getInput() {
		//Get Sticks
		input_left_stick_y = driver_controller.GetY(
				GenericHID::JoystickHand::kLeftHand);
		input_right_stick_y = driver_controller.GetY(
				GenericHID::JoystickHand::kRightHand);
		input_left_stick_x = driver_controller.GetX(
				GenericHID::JoystickHand::kLeftHand);
		input_right_stick_x = driver_controller.GetX(
				GenericHID::JoystickHand::kRightHand);

		//Get Triggers
		input_lt = driver_controller.GetTriggerAxis(
				GenericHID::JoystickHand::kLeftHand);
		input_rt = driver_controller.GetTriggerAxis(
				GenericHID::JoystickHand::kRightHand);

		//Get Triggers
		input_b_lt = button_controller.GetTriggerAxis(
				GenericHID::JoystickHand::kLeftHand);
		input_b_rt = button_controller.GetTriggerAxis(
				GenericHID::JoystickHand::kRightHand);

		button_a = button_controller.GetAButton();
		button_b = button_controller.GetBButton();
		button_x = button_controller.GetXButton();
		button_y = button_controller.GetYButton();
		button_y_2 = button_y;

		//Get Buttons
		button_lb = button_controller.GetBumper(
				GenericHID::JoystickHand::kLeftHand);
		button_rb = button_controller.GetBumper(
				GenericHID::JoystickHand::kRightHand);

	}
};

START_ROBOT_CLASS(Robot)
