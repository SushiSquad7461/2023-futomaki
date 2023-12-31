// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.SmartDashboard.AllianceColor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Swerve swerve;

  private RobotContainer m_robotContainer;
  // TODO I think you could probably afford to cache this value longer than 20ms
  private AllianceColor color;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    swerve = Swerve.getInstance();
    color = AllianceColor.getInstance();
    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void robotPeriodic() { 
    CommandScheduler.getInstance().run();
    color.updateAllianceColor();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() { swerve.updateEncoders(); }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() { CommandScheduler.getInstance().cancelAll(); }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
