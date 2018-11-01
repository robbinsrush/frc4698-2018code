/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Including some things
// Dont exactly remember this, but I think we use this for the drivetrain
// controlling
#include <Drive/DifferentialDrive.h>
// The type of robot we are developing
// https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/241853-choosing-a-base-class
#include <IterativeRobot.h>
// These are for the Xbox controllers
#include <Joystick.h>
#include <XboxController.h>
// The motor controller libraries
#include <Spark.h>
#include <VictorSP.h>
// Allows us to turn multiple motors into one single group
#include <SpeedControllerGroup.h>
// The main FRC library
#include "WPILib.h"
// This was for when we tried to use that one brushless motor. Rest in pieces.
// #include <NidecBrushless.h>

// Controlling the solenoid
#include <DoubleSolenoid.h>

// Used during the autonomous section to keep track of time.
#include <Timer.h>
// Unused. For motor encoders.
#include <Encoder.h>

class Robot : public frc::IterativeRobot {
  // Here we turn every motor on a pin into an object
  // Then turn the left/right groups into a SpeedControllerGroup class
  // And hand the groups over to DifferentialController instead of each motor
  // individually.

  // The new gyro Finn and Rush set up.
  // Connected with I2C.
  frc::ADXRS450_Gyro gyro;

  // Unused. encoder.
  frc::Encoder encoder_left{9, 8, false, Encoder::EncodingType::k4X};

  // Timer for auto.
  frc::Timer m_timer;

  // All motor controllers, and their pins on the RoboRIO
  frc::VictorSP m_arm_left{6};
  frc::VictorSP m_arm_right{7};

  frc::VictorSP m_elevator_1{5};
  frc::VictorSP m_elevator_2{4};

  frc::VictorSP m_leftFrontMotor{0};
  frc::VictorSP m_leftRearMotor{1};

  frc::SpeedControllerGroup m_left{m_leftFrontMotor, m_leftRearMotor};

  frc::VictorSP m_rightFrontMotor{2};
  frc::VictorSP m_rightRearMotor{3};

  frc::SpeedControllerGroup m_right{m_rightFrontMotor, m_rightRearMotor};

  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  // Unused.
  frc::Joystick m_stick{0};

  // Initiating the XBox controller(s)

  frc::XboxController driver_controller{0};
  frc::XboxController button_controller{1};

  // Ultrasonic sensor. Unused
  frc::AnalogInput ultra_1{0};

 public:
  // When the robot is first booting up
  void RobotInit() {
    // Starting the timer
    m_timer.Start();
    // Starting the USB Webcam
    CameraServer::GetInstance()->StartAutomaticCapture();
    // Connecting to the SmartDashboard
    // The smartdashboard is where we set/get settings.
    prefs = Preferences::GetInstance();

    // This is where we enable the Pneumatics if the setting is true
    // Command breakdown:
    // pneumatics_enable: the variable we are setting
    // prefs: the smartdashboard object we can talk to
    // prefs->GetBoolean: GetBoolean is a function of prefs, and is called with
    // a -> "PneumaticsEnable": Is the name of the value we are looking for
    // false: the default in case there is no value set already
    // This applies to all similar commands
    pneumatics_enable = prefs->GetBoolean("PneumaticsEnable", false);
    if (pneumatics_enable) {
      dblSolenoid = new DoubleSolenoid(0, 1);
    }

    // Unused. Resets the left encoder
    encoder_left.Reset();
  }

  // Ran one once when the robot is disables.
  void DisabledInit() { m_robotDrive.ArcadeDrive(0.0, 0.0); }

  // Ran over and over and over when the robot is disabled
  void DisabledPeriodic() {
    // Getting the many multipliers for driving
    drive_multiplier = prefs->GetDouble("DriveMultiplier", 0.75);
    turn_multiplier = prefs->GetDouble("TurnMultiplier", 0.70);
    climber_multiplier = prefs->GetDouble("ClimberMultiplier", 1);
    ramp_multiplier = prefs->GetDouble("RampMultiplier", 0.03);

    // Values for the non-driver player
    climber_hold = prefs->GetDouble("ClimberHold", 0.10);
    box_motor = prefs->GetDouble("BoxMotor", 0.5);

    // Snapping values for the teletop driving algorithm.
    // These will be explained once we get to that
    snap_distance_top = prefs->GetDouble("SnapDistanceTop", 0.05);
    snap_distance_bottom = prefs->GetDouble("SnapDistanceBottom", 0.2);

    // Values for autonomous.
    auto_time = prefs->GetInt("AutoTime", 1);
    auto_amount = prefs->GetDouble("AutoAmount", 0.01);
    gyrokp = prefs->GetFloat("gyrokp", 0.03);
    gyro_enable = prefs->GetBoolean("GyroEnable", false);
    mid_auto = prefs->GetBoolean("MidAuto", true);

    // Show preferences in SmartDashboard (mainly for debugging)
    SmartDashboard::PutNumber("Drive Multiplier:", drive_multiplier);
    SmartDashboard::PutNumber("Turn Multiplier:", turn_multiplier);
    SmartDashboard::PutNumber("Climber Multiplier:", climber_multiplier);
    SmartDashboard::PutNumber("Snap Distance Top:", snap_distance_top);
    SmartDashboard::PutNumber("Snap Distance Bottom:", snap_distance_bottom);
    SmartDashboard::PutNumber("Ramp Multiplier:", ramp_multiplier);
    SmartDashboard::PutBoolean("Pneumatics Enabled:", pneumatics_enable);
    SmartDashboard::PutNumber("Auto Time:", auto_time);
    SmartDashboard::PutNumber("Auto Amount:", auto_amount);
    SmartDashboard::PutNumber("Box Motor Multiplier:", box_motor);
    SmartDashboard::PutNumber("Gyro kP:", gyrokp);
    SmartDashboard::PutNumber("Gyro Enable:", gyro_enable);
    SmartDashboard::PutNumber("Climber Hold:", climber_hold);
    SmartDashboard::PutBoolean("Middle Auto:", mid_auto);

    // Just in case commands:
    // Set the robot to not move
    // Set the elevator to hold it's position
    m_robotDrive.ArcadeDrive(0.0, 0.0);
    m_elevator_1.Set(-climber_hold);

    // Wait for a lil bit to let the processor calm down
    Wait(1);
  }

  // My disgusting way of getting autonomous done.
  // Ok so let's discuss how I did this
  /*

  So this thing is weird.

  Basically, it's a GIANT IF statement with a lot of else ifs

  It checks to see if the time (for future reference, by time I mean the time
  since Auto has started.) is less than getWaitTime's response to the given
  stage number For example: getWaitTime(5)

  So if it is less than the returned time by the function, it will run that code
  It checks this code over and over again until it is larger than that stage's
  time, and moves on

  Example stage lengths:
  Stage 0: 2 seconds
  Stage 1: 5 seconds
  Stage 2: 0.1 seconds
  Stage 3: 4 seconds
  Stage 4: 1 second

  Example 1:
  The time is at 1 second, and stage 0 is 2 seconds long
  Thus, stage 0 has 1 second left and will keep being ran

  Example 2:
  The time is at 3 seconds
  The function getWaitTime adds together stage 0 and 1 to get 7 seconds.
  Thus, stage 1 has 4 seconds to go.


  */
  float getWaitTime(int stage) {
    if (stage == 0) {
      return stageLength[0];
    } else {
      float addingTime;
      for (int i = 0; i < stage + 1; i++) {
        addingTime += stageLength[i];
      }
      return addingTime;
    }
  }

  // Ran for one time after auto starts
  void AutonomousInit() {
    // Gets the direction our side's switch is.
    std::string gameData;
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    if (gameData.length() > 0) {
      if (gameData[0] == 'L') {
        // If it's on the left, set this to -1
        leftRightNegative = -1;
      } else {
        // Else, set it to 1
        leftRightNegative = 1;
      }
    }
    // That previous leftRightNegative variable is used to reuse the same code,
    // and just changing the direction the robot turns.

    // Set the gyro's current position to 0
    gyro.Reset();

    // Start the timer over again to 0
    m_timer.Reset();
    m_timer.Start();

    // If the pneumatics are on, make sure the box is being held
    if (pneumatics_enable) {
      dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
    }

    // Setting up the stages and their lengths
    if (mid_auto) {
      stageLength[0] = 2;
      // Stage 0: Raise the elevator
      stageLength[1] = 1;
      // Stage 1: Drive forward
      stageLength[2] = 0.1;
      // Stage 2: Hold the elevator up and reset gyro
      stageLength[3] = 2;
      // Stage 3: Rotate
      stageLength[4] = 0.2;
      // Stage 4: Reset gyro
      stageLength[5] = 1.4;
      // Stage 5: Forward
      stageLength[6] = 2;
      // Stage 6: Rotate
      stageLength[7] = 0.1;
      // Stage 7: Reset Gyro
      stageLength[8] = 1.5;
      // Stage 8: Forward
      stageLength[9] = 1;
      // Stage 9: Spit box
    } else {
      stageLength[0] = 6;
      // Just go forward
      // Dont quite know where the code for turning and spitting went.
    }
  }

  // This code is run over and over while the robot is in autonomous mode.
  void AutonomousPeriodic() {
    // Makes sure the elevator doesn't drop the box by stalling it upwards.
    m_elevator_1.Set(climber_hold);

    // Get the gyro's angle and output to the SmartDashboard
    float angle = -gyro.GetAngle();
    SmartDashboard::PutNumber("Gyro Angle:", angle);

    // Set up the the variables for correction using the gyro
    float targetHeading;
    float error;

    // Get the current time in auto mode
    float time = m_timer.Get();

    // When we're in the middle
    if (mid_auto) {
      if (time < getWaitTime(0)) {
        // Stage 0: Raise the elevator
        m_elevator_1.Set(-0.75);
        SmartDashboard::PutNumber("Auto Stage:", 0);
      } else if (time < getWaitTime(1)) {
        // Stage 1: Drive forward
        m_robotDrive.ArcadeDrive(0.7,
                                 -angle * gyrokp);  // turn to correct heading
        SmartDashboard::PutNumber("Auto Stage:", 1);
        Wait(0.004);
      } else if (time < getWaitTime(2)) {
        // Stage 2: Hold the elevator up and reset gyro
        m_elevator_1.Set(-climber_hold);
        gyro.Reset();
      } else if (time < getWaitTime(3)) {
        // Stage 3: Rotate
        targetHeading = leftRightNegative * 90;
        error = angle - targetHeading;
        angle = -gyro.GetAngle();
        if (abs(error) < 35) {
          SmartDashboard::PutNumber("Auto Stage:", 2.5);
          m_robotDrive.ArcadeDrive(
              0.0, leftRightNegative * 0.075 * -gyrokp * (error));
          SmartDashboard::PutNumber("Gyro Kp result:",
                                    (0.26 * -gyrokp * (error)));
        } else {
          SmartDashboard::PutNumber("Auto Stage:", 2);
          m_robotDrive.ArcadeDrive(0.0, leftRightNegative * 0.5);
        }
      } else if (time < getWaitTime(4)) {
        // Stage 4: Reset gyro
        SmartDashboard::PutNumber("Auto Stage:", 3);
        gyro.Reset();
      } else if (time < getWaitTime(5)) {
        // Stage 5: Forward
        SmartDashboard::PutNumber("Auto Stage:", 4);
        m_robotDrive.ArcadeDrive(0.7,
                                 -angle * gyrokp);  // turn to correct heading
        Wait(0.004);
      } else if (time < getWaitTime(6)) {
        // Stage 6: Rotate
        targetHeading = leftRightNegative * -90;
        error = angle - targetHeading;
        angle = -gyro.GetAngle();
        if (abs(error) < 35) {
          SmartDashboard::PutNumber("Auto Stage:", 5.5);
          m_robotDrive.ArcadeDrive(
              0.0, leftRightNegative * -0.075 * gyrokp * (error));
          SmartDashboard::PutNumber("Gyro Kp result:",
                                    (0.26 * -gyrokp * (error)));
        } else {
          SmartDashboard::PutNumber("Auto Stage:", 5);
          m_robotDrive.ArcadeDrive(0.0, leftRightNegative * -0.5);
        }
      } else if (time < getWaitTime(7)) {
        // Stage 7: Reset Gyro
        SmartDashboard::PutNumber("Auto Stage:", 6);
        gyro.Reset();
      } else if (time < getWaitTime(8)) {
        // Stage 8: Forward
        SmartDashboard::PutNumber("Auto Stage:", 4);
        m_robotDrive.ArcadeDrive(0.7,
                                 -angle * gyrokp);  // turn to correct heading

        Wait(0.004);
      } else if (time < getWaitTime(9)) {
        // Stage 9: Spit box
        SmartDashboard::PutNumber("Auto Stage:", 6);
        m_arm_left.Set(0.6);
        m_arm_right.Set(-0.6);
      } else {
        m_arm_left.Set(0);
        m_arm_right.Set(0);
      }
    } else {
      if (time < getWaitTime(0)) {
      	// Just go forward
        SmartDashboard::PutNumber("Auto Stage:", 0);
        m_robotDrive.ArcadeDrive(0.55,
                                 -angle * gyrokp);  // turn to correct heading
        SmartDashboard::PutNumber("Gyro Kp result:", -angle * gyrokp);
        Wait(0.004);
      }
    }

    // SmartDashboard::PutNumber("Gyro Angle:", angle);
  }


  // Ran once time when robot is set to teleop
  void TeleopInit() {
  	// Makes sure the pneumatics are closed
    if (pneumatics_enable) {
      dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
    }
  }

  // Ran forever while in teletop
  void TeleopPeriodic() {
  	// Gets the current inputs of the Xbox controllers
    getInput();

    // This is a button the secondary driver can use to force the primary driver to slow down.
    // Button Y on second controller
    if (button_y_2) {
      if (force_slow > 0 && force_slow < 0.4) {
        force_slow -= 0.1;
      }
    } else {
      if (force_slow < 1) {
        force_slow += 0.1;
      }
    }

    // This next chunk is courtesy of Danny.
    // This is for having the inputs of the driver ramp up and down, because going from 0 to 100 back to 0 instantly is NOT SAFE AND IS NOT GOOD. IM LOOKING AT YOU, SHANE.

    // We want to make current value get closer to the target, which is this.
    target = (input_lt - input_rt) * drive_multiplier * force_slow;
    // If it's close enough to being the target value, just let it happen.
    if (target - current < snap_distance_top &&
        target - current > -snap_distance_top) {
      current = target;
    }
    // When there's no input, start slowing down the bus.
    if (input_lt == 0.0 && input_rt == 0.0) {  
      current += (target - current) * (ramp_multiplier + 0.1);
    } else {
      // Otherwise, start adding little bits of the delta between the target and
      // the current to the current value 
   	  // In english,
      // Start adding what's missing from the current value slowly.
      current += (target - current) * ramp_multiplier;

      // If in this limbo area, snap out of it!
		if (current < 0 && current > -snap_distance_bottom) {
			if (input_rt != 0.0) {
				// Right Trigger held, pull up
				current = -snap_distance_bottom;
			} else {
				current = 0;
				// Right trigger isn't held, snap down to zero
			}
		} else if (current > 0 && current < snap_distance_bottom) {
			if (input_lt != 0.0) {  
				// left Trigger held, pull up
			    current = snap_distance_bottom;
			} else {  
				// Left trigger isn't held, snap down to zero
          		current = 0;
			}
		}
    }

    // Used to toggle holding the elevator up when it's not being controlled
    // Button X on the secondary controller
    if (button_x_prev != button_x && button_x) {
      holdClimber = !holdClimber;
    }

    // Secondary controller's elevator buttons:
    // Right Bumper is down
    // Left Bumper is up
    // (I think)
    if (button_rb) {
      m_elevator_1.Set(-climber_multiplier);
      holdClimber = true;
    } else if (button_lb) {
      // Makes the elevator go a quarter of fast in this direction
      m_elevator_1.Set(climber_multiplier / 4);
      holdClimber = true;
    } else if (holdClimber) {
      // Hold it up when it's not being controller
      m_elevator_1.Set(climber_hold);
      holdClimber = true;
    } else {
      m_elevator_1.Set(0);
    }

    // A part of the elevator hold toggle.
    button_x_prev = button_x;

    // !IMPORTANT!
    // This is sending the end result of the algorithm to the motors. 
    m_robotDrive.ArcadeDrive(-current, input_left_stick_x * turn_multiplier);

    // Put on dashboard
    SmartDashboard::PutNumber("Target Speed:", -target);
    SmartDashboard::PutNumber("Current Speed:", -current);
    SmartDashboard::PutNumber("Current Stick:", input_left_stick_x);
    SmartDashboard::PutNumber("Current Turn:", input_left_stick_x * (-target));
    SmartDashboard::PutNumber("Ultrasonic 1:",
                              (ultra_1.GetVoltage() / 0.001) / 45);
    count_left = encoder_left.Get();
    SmartDashboard::PutNumber("Counter Left", count_left);
    SmartDashboard::PutNumber("Force Slow", force_slow);

    // Pneumatics controls for the secondary controller
    // A is open
    // B is close
    if (pneumatics_enable) {
      if (button_a) {
        dblSolenoid->Set(DoubleSolenoid::Value::kForward);
      }
      if (button_b) {
        dblSolenoid->Set(DoubleSolenoid::Value::kReverse);
      }
    }

    // Trigger controls for the secondary controller to control the elevator's box motors
    if (input_b_rt != 0) {
      m_arm_left.Set(box_motor * -input_b_rt);
      m_arm_right.Set(-box_motor * -input_b_rt);
    } else if (input_b_lt != 0) {
      m_arm_left.Set(-box_motor * -input_b_lt);
      m_arm_right.Set(box_motor * -input_b_lt);
    } else {
      m_arm_left.Set(0.0);
      m_arm_right.Set(0.0);
    }
  }

 private:
  bool pneumatics_enable;
  // Robot Preferences from SmartDashboard
  Preferences *prefs;
  // Value from 0.0 to 1.0, reduces motor speed to prevent issues during
  // testing.
  double drive_multiplier;      // Default 0.75
  double snap_distance_top;     // Default is 0.05
  double snap_distance_bottom;  // Default is 0.2
  double ramp_multiplier;       // Default is 0.03
  double turn_multiplier;       // Default 0.50
  double climber_multiplier;    // Default 0.3
  double box_motor;
  double climber_hold;

  int time, time_2 = 0;

  // Autonomous Values
  double auto_time;
  double auto_amount;

  float gyrokp;
  int leftRightNegative = 0;
  float stageLength[10];
  bool mid_auto;

  // Pneumatics
  DoubleSolenoid *dblSolenoid;

  // Drive Values
  double current = 0.0, target = 0.0;

  float force_slow = 1.0;

  // Analog stick input values
  double input_left_stick_y, input_right_stick_y, input_left_stick_x,
      input_right_stick_x;

  // Trigger Values
  double input_lt, input_rt, input_b_lt, input_b_rt;

  // Face button values
  bool button_a, button_b, button_x, button_y, button_y_2;

  bool button_x_prev, holdClimber, ignoreNext = false;

  // Bumpers

  bool button_lb, button_rb;

  bool gyro_enable;

  int count_left;

  // Encoder *sampleEncoder;

  void getInput() {
    // Get Sticks
    input_left_stick_y =
        driver_controller.GetY(GenericHID::JoystickHand::kLeftHand);
    input_right_stick_y =
        driver_controller.GetY(GenericHID::JoystickHand::kRightHand);
    input_left_stick_x =
        driver_controller.GetX(GenericHID::JoystickHand::kLeftHand);
    input_right_stick_x =
        driver_controller.GetX(GenericHID::JoystickHand::kRightHand);

    // Get Triggers
    input_lt =
        driver_controller.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand);
    input_rt =
        driver_controller.GetTriggerAxis(GenericHID::JoystickHand::kRightHand);

    // Get Triggers
    input_b_lt =
        button_controller.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand);
    input_b_rt =
        button_controller.GetTriggerAxis(GenericHID::JoystickHand::kRightHand);

    button_a = button_controller.GetAButton();
    button_b = button_controller.GetBButton();
    button_x = button_controller.GetXButton();
    button_y = button_controller.GetYButton();
    button_y_2 = button_y;

    // Get Buttons
    button_lb =
        button_controller.GetBumper(GenericHID::JoystickHand::kLeftHand);
    button_rb =
        button_controller.GetBumper(GenericHID::JoystickHand::kRightHand);
  }
};

START_ROBOT_CLASS(Robot)
