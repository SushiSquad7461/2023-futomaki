// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Swerve.SDSModules;
import SushiFrcLib.Swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.FutomakiSwerveModule;
import frc.robot.util.SwerveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean kTuningMode = true;
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
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0.2; // properly tuned
    }

    public static final class kSwerve {
        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(28);
        public static final double WHEEL_BASE = Units.inchesToMeters(28);
        public static final double WHEEL_DIAMATER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMATER * Math.PI;

        public static final SwerveKinematics SWERVE_KINEMATICS = new SwerveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 20;
        public static final int DRIVE_CURRENT_LIMIT = 40;

        /* Angle Motor PID Values */
        public static final double ANGLE_P = 0.005; // 0.3
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 0.0;
        public static final double ANGLE_F = 0.0; // 12.0

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.01700; // 0.009
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.0458;

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 5; // 4.5 meters per second
        public static final double MAX_ACCELERATION = 4; // 2
        public static final double MAX_ANGULAR_VELOCITY = 10; // 11.5
        public static final double MAX_ANGULAR_ACCELERATION = 20; // 11.5

        /* Motor Inverts */
        public static final boolean DRIVE_INVERSION = false;
        public static final boolean ANGLE_INVERSION = false; // make true if we have a stroke

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERSION = true;

        public static final boolean SWERVE_TUNNING_MODE = false;

        public static final SwerveModuleConstants MOD0_CONSTANTS = new FutomakiSwerveModule(0, 249.9, SDSModules.MK4i);
        public static final SwerveModuleConstants MOD1_CONSTANTS = new FutomakiSwerveModule(1, 68.291, SDSModules.MK4i);
        public static final SwerveModuleConstants MOD2_CONSTANTS = new FutomakiSwerveModule(2, 268.33, SDSModules.MK4i);
        public static final SwerveModuleConstants MOD3_CONSTANTS = new FutomakiSwerveModule(3, 194.58, SDSModules.MK4i);
    }

  public static class kManipulator {
    public static final double kP = 0.03;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;

    public static final double kG = 0.0;
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 0;

    public static final int kSpinMotorID = 24;
    public static final int kPositionMotorID = 21;

    public static final double MANIPULATOR_GEAR_RATIO = 160/3; //160:3
    public static final int ENCODER_CHANNEL = 5;
    public static final double ENCODER_ANGLE_OFFSET = (-74.6);

    public static final int SPIN_CURRENT_LIMIT = 25;
    public static final int POSITION_CURRENT_LIMIT = 20;

    public static final int REFERENCE_VAL = 0;
    public static final int TUNE_HIGH_VAL = 100;
    public static final int TUNE_LOW_VAL = -30;

    public static final double WRIST_SPEED = 1.0;
    public static final double WRIST_REVERSE_SPEED = WRIST_SPEED*-1;
    public static final double WRIST_STOP_SPEED = 1.0;

    public static final int ERROR_LIMIT = 1;

    public static final int PID_SLOT = 0;
    public static final int WRIST_FEED_FORWARD_VELOCITY = 0;
    public static final int WRIST_FEED_FORWARD_ACCEL = 0;

  }
  public enum RobotState {
    IDLE(3, 80, 0.0), 
    GROUND_CONE(3.5, 0, -1.0),
    GROUND_CUBE(3, 0, 1.0),
    DOUBLE_CONE(0, 0, 0),
    SINGLE_CONE(65,2,-1.0),
    L1_CUBE(5,0,0),
    L2_CUBE(25,0,0),
    L3_CUBE(45,0,0),
    L1_CONE(0,0,0),
    L2_CONE(0,0,0),
    L3_CONE(45,-20,-0.5);


    public double elevatorPos;
    public double wristPos;
    private double manipulatorSpeed;

    private RobotState(double elevatorPos, double wristPos, double manipulatorSpeed) {
        this.elevatorPos = elevatorPos;
        this.wristPos = wristPos;
        this.manipulatorSpeed = manipulatorSpeed;
    }

    public double getElevatorPos() {return elevatorPos; }
    public double getWristPos() {return wristPos; }
    public double getManipulatorSpeed() {return manipulatorSpeed;}
  }
}
