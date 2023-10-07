// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotState;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.BuddyClimb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final OI oi;
  private final Manipulator manipulator;
  private final Elevator elevator;

  public RobotContainer() {                     
    oi = OI.getInstance();
    elevator = Elevator.getInstance();
    manipulator = Manipulator.getInstance();
    configureBindings();

    new BuddyClimb();
  }

  private void configureBindings() {
    Swerve.getInstance().setDefaultCommand(
      new TeleopSwerveDrive(
          Swerve.getInstance(), 
          () -> oi.getDriveTrainTranslationX(),
          () -> oi.getDriveTrainTranslationY(),
          () -> oi.getDriveTrainRotation()
      )
    );

    oi.getDriverController().y().onTrue(manipulator.runWristForward());
    oi.getDriverController().a().onTrue(manipulator.runWristBackward());
    oi.getDriverController().x().onTrue(manipulator.stopWristBackward());

    oi.getDriverController().b().onTrue(setRobotState(RobotState.GROUND_CONE));
  }

  private Command setRobotState(Constants.RobotState state) {
    return new InstantCommand(() -> {
      elevator.pid(state.elevatorPos);
      manipulator.runWrist(state.getManipulatorSpeed());
      manipulator.setAngle(state.wristPos);
    }, elevator, manipulator);
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
