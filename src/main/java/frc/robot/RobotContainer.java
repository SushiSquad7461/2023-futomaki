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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
  private final SendableChooser<Command> scoreChooser;

  public RobotContainer() {                     
    oi = OI.getInstance();
    elevator = Elevator.getInstance();
    manipulator = Manipulator.getInstance();
    scoreChooser = new SendableChooser<Command>();

    configureBindings();

    new BuddyClimb();
  }

  private void configureBindings() {

    scoreChooser.addOption("L3 Cone", setRobotState(RobotState.L3_CONE));
    scoreChooser.addOption("L3 Cube", setRobotState(RobotState.L3_CUBE));
    scoreChooser.addOption("L2 Cone", setRobotState(RobotState.L2_CONE));
    scoreChooser.addOption("L2 Cube", setRobotState(RobotState.L2_CUBE));
    scoreChooser.addOption("L1 Cone", setRobotState(RobotState.L1_CONE));
    scoreChooser.addOption("L1 Cube", setRobotState(RobotState.L1_CUBE));

    Swerve.getInstance().setDefaultCommand(
      new TeleopSwerveDrive(
          Swerve.getInstance(), 
          () -> oi.getDriveTrainTranslationX(),
          () -> oi.getDriveTrainTranslationY(),
          () -> oi.getDriveTrainRotation()
      )
    );

    // Their are 5 driver commands
    // Ground cone pickup
    // Ground cube pickup
    // Single Suby pickup cone
    // Double Suby pickup cone
    // Score

    // oi.getDriverController().y().onTrue(manipulator.runWristForward());
    // oi.getDriverController().a().onTrue(manipulator.runWristBackward());
    // oi.getDriverController().x().onTrue(manipulator.stopWristBackward());

    oi.getDriverController().b().whileTrue(setRobotState(RobotState.GROUND_CONE)).onFalse(setRobotState(RobotState.IDLE));
    oi.getDriverController().a().whileTrue(setRobotState(RobotState.GROUND_CUBE)).onFalse(setRobotState(RobotState.IDLE));
    oi.getDriverController().x().whileTrue(setRobotState(RobotState.SINGLE_CONE)).onFalse(setRobotState(RobotState.IDLE));
    oi.getDriverController().y().whileTrue(setRobotState(RobotState.DOUBLE_CONE)).onFalse(setRobotState(RobotState.IDLE));

    oi.getDriverController().povUp().whileTrue(scoreChooser.getSelected()).onFalse(new SequentialCommandGroup(
      manipulator.reverseCurrentWrist(),
      new WaitCommand(1.0),
      setRobotState(RobotState.IDLE)
    ));
  }

  private Command setRobotState(Constants.RobotState state) {
    return new InstantCommand(() -> {
        if (state.getElevatorPos() > elevator.getPose() && state.getWristPos() < manipulator.getWristPos()) {
          new SequentialCommandGroup(
            elevator.pid(state.elevatorPos),
            manipulator.setPosition(state.wristPos),
            manipulator.runWrist(state.getManipulatorSpeed())
          ).schedule();  
        } else {
          new SequentialCommandGroup(
            manipulator.setPosition(state.wristPos),
            elevator.pid(state.elevatorPos),
            manipulator.runWrist(state.getManipulatorSpeed())
          ).schedule(); 
        }
      }
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
