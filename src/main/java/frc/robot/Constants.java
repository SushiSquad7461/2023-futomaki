// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import SushiFrcLib.Swerve.SDSModules;
import SushiFrcLib.Swerve.SwerveKinematics;
import SushiFrcLib.Swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.FutomakiSwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean kTuningMode = false;
    public static final double STICK_DEADBAND = 0.1;

    public static final class kOI {
        public static final int DRIVE_TRANSLATION_Y = XboxController.Axis.kLeftY.value;
        public static final int DRIVE_TRANSLATION_X = XboxController.Axis.kLeftX.value;
        public static final int DRIVE_ROTATE = XboxController.Axis.kRightX.value;

        public static final int UPDATE_ENCODER = XboxController.Button.kY.value;

        public static final int DRIVE_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static class kPorts {
        public static final String CANIVORE_NAME = "Sussy Squad";
        public static final int PIGEON_ID = 13;
    }

    public static class kElevator {
        public static final double kP_UP = 0.1; // 0.03
        public static final double kP_DOWN = 0.04;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG_DOWN = 0.3;
        public static final double kG_UP = 0.7;

        public static final int LEFT_MOTOR_ID = 22;
        public static final int RIGHT_MOTOR_ID = 20;

        public static final int MAX_POS = 50;
        public static final int MIN_POS = 0;
        public static final double DEFUALT_VAL = RobotState.IDLE.elevatorPos;

        public static final int CURRENT_LIMIT = 40;
    }
    
    public static final class kBuddyClimb {
      public static final int BUDDY_CLIMB_MOTOR_ID = 30;
    }

    public static final class kSwerve {
        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(28);
        public static final double WHEEL_BASE = Units.inchesToMeters(28);
        public static final double WHEEL_DIAMATER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMATER * Math.PI;

        public static final SwerveKinematics SWERVE_KINEMATICS = new SwerveKinematics(
          new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
        );

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 20;
        public static final int DRIVE_CURRENT_LIMIT = 60;

        /* Translation Constants */
        public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(3.5,0,0);

        /* Rotation Constants */
        public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(3.0,0,0);

        /* Angle Motor PID Values */
        public static final double ANGLE_P = 0.1; // 0.3
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 0.0;
        public static final double ANGLE_F = 0.0; // 12.0

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.1; // 0.025
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.0458; // 0.04

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 5; // 5 meters per second
        public static final double MAX_ACCELERATION = 4; // 4
        public static final double MAX_ANGULAR_VELOCITY = 10; // 11.5
        public static final double MAX_ANGULAR_ACCELERATION = 20; // 11.5

        /* Motor Inverts */
        public static final boolean DRIVE_INVERSION = false;
        public static final boolean ANGLE_INVERSION = false; // make true if we have a stroke

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERSION = true;

        public static final boolean SWERVE_TUNNING_MODE = false;

        public static final SwerveModuleConstants MOD0_CONSTANTS = new FutomakiSwerveModule(0, 159.9, SDSModules.MK4i);
        public static final SwerveModuleConstants MOD1_CONSTANTS = new FutomakiSwerveModule(1, 338.291, SDSModules.MK4i);
        public static final SwerveModuleConstants MOD2_CONSTANTS = new FutomakiSwerveModule(2, 178.33, SDSModules.MK4i);
        public static final SwerveModuleConstants MOD3_CONSTANTS = new FutomakiSwerveModule(3, 104.58, SDSModules.MK4i);
    }

  public static class kManipulator {
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kG_UP = 0.8; //0.2
    public static final double kG_DOWN = 0.2;

    public static final double kP_UP = 0.02; //set this
    public static final double kP_DOWN = 0.015; //set this

    public static final int kSpinMotorID = 24;
    public static final int kPositionMotorID = 21;

    public static final double MANIPULATOR_GEAR_RATIO = 160/3; //160:3
    public static final int ENCODER_CHANNEL = 5;
    public static final double ENCODER_ANGLE_OFFSET = -77.100446;

    public static final int SPIN_CURRENT_LIMIT = 30;
    public static final int POSITION_CURRENT_LIMIT = 40;

    public static final double DEFUALT_VAL = RobotState.IDLE.wristPos;
    public static final double TUNE_HIGH_VAL = 100;
    public static final double TUNE_LOW_VAL = -30;

    public static final double WRIST_SPEED = 1.0;
    public static final double WRIST_REVERSE_SPEED = WRIST_SPEED*-1;
    public static final double WRIST_STOP_SPEED = 1.0;

    public static final int ERROR_LIMIT = 1;
    public static final int MAX_ERROR = 5;
  }

  public static class kAuto {
    public static final double CHARGE_SPEED = 1.5; // meters per second
    public static final double AUTO_BALANCE_WAIT = 0.5;

    public static final double BURM_SIDE_SPEED = 2.0; //mps
    public static final double CUBE_SCORE_WAIT = 0.3; // sec
}

  public static class kAutoBalance {
    public static final double FLATNESS_THRESHOLD_DEGREES = 0.15;
    public static final double MAX_TILT_CHANGE_DIVIDER = 10; // TODO: Name better
    public static final double MAX_SPEED = Constants.kSwerve.MAX_SPEED * 0.0085;
  }

  public enum RobotState {
    IDLE(3, 80, 0, false), 
    GROUND_CONE(12, -39, -1.0),
    GROUND_CUBE(3, 0, 1.0),
    DOUBLE_CONE(0, 0, 0),
    SINGLE_CONE(0.8,70,-1.0),
    L1_CUBE(3,80,1),
    L2_CUBE(25,15,0.5),
    L3_CUBE(45,25,0.5),
    L1_CONE(3,60,-1.0),
    L2_CONE(18.8,23,-1.0
    ), // 45 -70
    L3_CONE(45,-10,-0.5);


    public double elevatorPos;
    public double wristPos;
    public double manipulatorSpeed;
    public boolean changeSpeed;

    private RobotState(double elevatorPos, double wristPos, double manipulatorSpeed) {
      this(elevatorPos, wristPos, manipulatorSpeed, true);
    }

    private RobotState(double elevatorPos, double wristPos, double manipulatorSpeed, boolean changeSpeed) {
        this.elevatorPos = elevatorPos;
        this.wristPos = wristPos;
        this.manipulatorSpeed = manipulatorSpeed;
        this.changeSpeed = changeSpeed; 
    }
  }
}
