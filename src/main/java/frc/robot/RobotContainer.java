// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final OI oi;
  // private final Elevator elavator;

  public RobotContainer() {                     
    // elavator = Elevator.getInstance();
    // new Manipulator();
    oi = OI.getInstance();
    Elevator.getInstance();
    configureBindings();
  }

  private void configureBindings() {
    // elavator.setDefaultCommand(new InstantCommand(() -> elavator.run(OI.getInstance().getDriverController().getLeftY()/2), elavator));
    Swerve.getInstance().setDefaultCommand(
      new TeleopSwerveDrive(
          Swerve.getInstance(), 
          () -> oi.getDriveTrainTranslationX(),
          () -> oi.getDriveTrainTranslationY(),
          () -> oi.getDriveTrainRotation()
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
