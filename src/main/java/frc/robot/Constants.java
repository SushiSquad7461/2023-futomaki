// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Swerve.SwerveModuleConstants;

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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class kElevator {
    public static final double kP = 0.04  ;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.4; // increase later properly tune
  }

  public static class kManipulator {
    public static final int kSpinMotorID = -1;
    public static final int kPositionMotorID = 24;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.05;
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 0;
  }

  
}
