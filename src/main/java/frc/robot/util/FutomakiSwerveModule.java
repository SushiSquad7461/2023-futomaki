package frc.robot.util;

import SushiFrcLib.Swerve.SwerveModuleConstants;

public class FutomakiSwerveModule extends SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderId;
    public final double angleOffset;

    public static int angleCurrentLimit;
    public static boolean angleInversion;

    public static double angleP;
    public static double angleI;
    public static double angleD;
    public static double angleF;

    public static int driveCurrentLimit;
    public static boolean driveInversion;

    public static double driveP;
    public static double driveI;
    public static double driveD;
    public static double driveF;

    public static double openLoopRamp;

    public static boolean cancoderInversion;
    public static String canivoreName;

    public static double maxSpeed;

    public FutomakiSwerveModule(int moduleNumber, double angleOffset, SDSModules moduleInfo) {
        super(moduleNumber, angleOffset, moduleInfo);
    }
}
