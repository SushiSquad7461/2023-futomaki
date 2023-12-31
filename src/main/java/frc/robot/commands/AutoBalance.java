// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import SushiFrcLib.Sensors.gyro.Pigeon;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kAutoBalance;
import frc.robot.subsystems.Swerve;

/**
 * Auto balances the robot using Sushi Squad panteted bang bang.
 */
public class AutoBalance extends CommandBase {
    Swerve swerve;
    Translation2d tilt;
    Translation2d initialTilt;

    public AutoBalance() {
        swerve = Swerve.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        initialTilt = new Translation2d(getRoll(), getPitch());
    }

    @Override
    public void execute() {
        tilt = new Translation2d(getRoll(), getPitch());

        SmartDashboard.putNumber("roll", getRoll());
        SmartDashboard.putNumber("pitch", getPitch());
        SmartDashboard.putNumber("autobalance norm", tilt.getNorm());

        if (!(
            (initialTilt.getNorm() - tilt.getNorm())
            > (initialTilt.getNorm() / kAutoBalance.MAX_TILT_CHANGE_DIVIDER))) {

            swerve.driveRobotOriented(tilt.times(kAutoBalance.MAX_SPEED), 0);
        } else {
            swerve.driveRobotOriented(new Translation2d(0, 0), 0.1);
        }
    }

    @Override
    public void end(boolean interrupted) { swerve.drive(new Translation2d(0, 0), 0.1); }

    @Override
    public boolean isFinished() { return tilt.getNorm() < kAutoBalance.FLATNESS_THRESHOLD_DEGREES; }

    // TODO casting is cringe -- you are basically negating the genericism of your swerve drive
    private double getRoll() { return ((Pigeon) swerve.getGyro()).getRoll(); }

    private double getPitch() { return ((Pigeon) swerve.getGyro()).getPitch(); }
}