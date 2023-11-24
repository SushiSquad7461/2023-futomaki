// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Motor.MotorConfig.Mode;
import SushiFrcLib.Swerve.SwerveKinematics;
import SushiFrcLib.Swerve.SwerveModules.SDSModules;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleConstants;
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
    public static final boolean TUNING_MODE = false;
    public static final RobotState DEFUAL_STATE = RobotState.IDLE;

    public static final class OI {
        public static final double STICK_DEADBAND = 0.1;
        public static final int UPDATE_ENCODER = XboxController.Button.kY.value;
    }

    public static class Ports {
        public static final String CANIVORE_NAME = "Sussy Squad";
        public static final int PIGEON_ID = 13;
    }

    public static class Elevator {
        public static final double P_UP = 0.1;
        public static final double P_DOWN = 0.04;
        public static final double G_DOWN = 0.3;
        public static final double G_UP = 0.7;

        public static final MotorConfig LEFT_MOTOR = new MotorConfig(
            22, 
            40, 
            false, 
            MotorConfig.Mode.BRAKE
        );

        public static final MotorConfig RIGHT_MOTOR = new MotorConfig(
            20, 
            40, 
            true, 
            PIDConfig.getPid(P_UP), 
            MotorConfig.Mode.BRAKE
        );

        public static final double MAX_ERROR = 5;
        public static final int MAX_POS = 50;
        public static final int MIN_POS = 0;
    }
    
    public static final class BuddyClimb {
        public static final int DOWN_CURRENT_LIMIT = 40;
        public static final int UP_CURRENT_LIMIT = 1;

        public static final MotorConfig MOTOR_CONFIG = new MotorConfig(
            30,
            DOWN_CURRENT_LIMIT,
            true,
            Mode.BRAKE
        );
    }

    public static final class Swerve {
        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(28);
        public static final double WHEEL_BASE = Units.inchesToMeters(28);
        public static final double WHEEL_DIAMATER = Units.inchesToMeters(4);

        public static final SwerveKinematics SWERVE_KINEMATICS = new SwerveKinematics(
          new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
        );


        public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
            20,
            false, // Make true if we have a stroke
            PIDConfig.getPid(0.1), 
            MotorConfig.Mode.BRAKE 
        );

        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
            60,
            false, 
            PIDConfig.getPid(0.1, 0.0458), 
            MotorConfig.Mode.BRAKE 
        );

        /* PathPlanner PID Constants */
        public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(3.5,0,0);
        public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(3.0,0,0);

        /* AutoRotate PID */
        public static final PIDConfig autoRotate = PIDConfig.getPid(0.1);

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 5; // 5 meters per second
        public static final double MAX_ACCELERATION = 4; // 4 meters per second squared
        public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI; // radians per second
        public static final double MAX_ANGULAR_ACCELERATION = 20; // TODO: tune

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERSION = true;

        public static final boolean SWERVE_TUNNING_MODE = false;

        public static final SwerveModuleConstants MOD0_CONSTANTS = new FutomakiSwerveModule(
            0, 
            159.9, 
            SDSModules.MK4i
        );

        public static final SwerveModuleConstants MOD1_CONSTANTS = new FutomakiSwerveModule(
            1, 
            338.291, 
            SDSModules.MK4i
        );

        public static final SwerveModuleConstants MOD2_CONSTANTS = new FutomakiSwerveModule(
            2, 
            178.33, 
            SDSModules.MK4i
        );

        public static final SwerveModuleConstants MOD3_CONSTANTS = new FutomakiSwerveModule(
            3, 
            104.58, 
            SDSModules.MK4i
        );
    }

  public static class Manipulator {
    public static final double G_UP = 0.8;
    public static final double G_DOWN = 0.2;

    public static final double P_UP = 0.02; 
    public static final double P_DOWN = 0.015;

    public static final MotorConfig SPIN_MOTOR = new MotorConfig(
        24,
        40,
        false,
        PIDConfig.getPid(P_UP),
        Mode.BRAKE
    );

    public static final MotorConfig POSITION_MOTOR = new MotorConfig(
        21,
        30,
        false,
        Mode.BRAKE
    );

    public static final double GEAR_RATIO = 160/3; //160:3
    public static final int ENCODER_CHANNEL = 5;
    public static final double ENCODER_ANGLE_OFFSET = -153.77;

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
    public static final double MAX_TILT_CHANGE_DIVIDER = 10;
    public static final double MAX_SPEED = Constants.Swerve.MAX_SPEED * 0.0085;
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
    L2_CONE(18.8,23,-1.0), 
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
