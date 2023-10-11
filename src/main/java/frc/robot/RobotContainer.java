// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotState;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.BuddyClimb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final Swerve swerve;

  private final SendableChooser<Command> scoreChooser;

  public RobotContainer() {                     
    oi = OI.getInstance();
    elevator = Elevator.getInstance();
    manipulator = Manipulator.getInstance();
    swerve = Swerve.getInstance();

    scoreChooser = new SendableChooser<Command>();
    setupScoreChooser();

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
      new TeleopSwerveDrive(
          Swerve.getInstance(), 
          () -> oi.getDriveTrainTranslationX(),
          () -> oi.getDriveTrainTranslationY(),
          () -> oi.getDriveTrainRotation(),
          () -> elevator.getPose() > 20 ? 0.2 : 1.0
      )
    );

    oi.getDriverController().b().onTrue(CommandFactory.setRobotState(manipulator, elevator, RobotState.GROUND_CONE)).onFalse(CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE));
    oi.getDriverController().a().onTrue(CommandFactory.setRobotState(manipulator, elevator, RobotState.GROUND_CUBE)).onFalse(CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE));
    oi.getDriverController().x().onTrue(CommandFactory.setRobotState(manipulator, elevator, RobotState.SINGLE_CONE)).onFalse(CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE));
    oi.getDriverController().y().onTrue(CommandFactory.setRobotState(manipulator, elevator, RobotState.DOUBLE_CONE)).onFalse(CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE));

    oi.getDriverController().povUp().onTrue(new InstantCommand(() -> scoreChooser.getSelected().schedule())).onFalse(new SequentialCommandGroup(
      manipulator.reverseCurrentWrist(),
      new WaitCommand(1.0),
      CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE)
    ));
  }

  private void setupScoreChooser() {
    scoreChooser.addOption("L3 Cone", CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE));
    scoreChooser.addOption("L3 Cube", CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CUBE));
    scoreChooser.addOption("L2 Cone", CommandFactory.setRobotState(manipulator, elevator, RobotState.L2_CONE));
    scoreChooser.addOption("L2 Cube", CommandFactory.setRobotState(manipulator, elevator, RobotState.L2_CUBE));
    scoreChooser.addOption("L1 Cone", CommandFactory.setRobotState(manipulator, elevator, RobotState.L1_CONE));
    scoreChooser.addOption("L1 Cube", CommandFactory.setRobotState(manipulator, elevator, RobotState.L1_CUBE));
    SmartDashboard.putData("Score Selecter", scoreChooser);
  }

  public Command getAutonomousCommand() { return null; }
}
