// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotState;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.BuddyClimb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import SushiFrcLib.Controllers.OI;
import SushiFrcLib.Swerve.TeleopSwerveDrive;
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
  private final AutoCommands autos;
  private final BuddyClimb climb;

  private final SendableChooser<Command> scoreChooser;

  public RobotContainer() {                     
    oi = OI.getInstance();
    elevator = Elevator.getInstance();
    manipulator = Manipulator.getInstance();
    swerve = Swerve.getInstance();
    autos = new AutoCommands(swerve, manipulator, elevator);
    climb = BuddyClimb.getInstance(); 

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
          () -> elevator.getPose() > (Constants.Elevator.MAX_POS / 2) ? Constants.OI.SLOW_SPEED : Constants.OI.FAST_SPEED
      )
    );

    oi.getDriverController().rightTrigger().onTrue(CommandFactory.setRobotState(manipulator, elevator, RobotState.GROUND_CONE)).onFalse(CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE));
    oi.getDriverController().leftTrigger().onTrue(CommandFactory.setRobotState(manipulator, elevator, RobotState.GROUND_CUBE)).onFalse(CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE));
    oi.getDriverController().rightBumper().onTrue(CommandFactory.setRobotState(manipulator, elevator, RobotState.SINGLE_CONE)).onFalse(CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE));

    oi.getDriverController().b().onTrue(new InstantCommand(() -> swerve.enableRotationLock(Constants.OI.SINGLE_SUBY_ALLIGMENT), swerve)).onFalse(new InstantCommand(() -> swerve.disableRotationLock(), swerve));
    oi.getDriverController().x().onTrue(new InstantCommand(() -> swerve.enableRotationLock(Constants.OI.GRID_ALLIGMENT), swerve)).onFalse(new InstantCommand(() -> swerve.disableRotationLock(), swerve));
    oi.getDriverController().povUp().onTrue(new InstantCommand(() -> swerve.enableRotationLock(Constants.OI.DEFUALT_ALLIGMENT), swerve)).onFalse(new InstantCommand(() -> swerve.disableRotationLock(), swerve));

    oi.getDriverController().povDown().whileTrue(new AutoBalance());

    // Score Command 
    oi.getDriverController().leftBumper().onTrue(new InstantCommand(() -> scoreChooser.getSelected().schedule())).onFalse(manipulator.reverseCurrentWrist().
      andThen(new WaitCommand(1.0)).
      andThen(CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE))
    );

    oi.getOperatorController().y().onTrue(elevator.resetElevatorPoseStart()).onFalse(elevator.resetElevatorPoseEnd());
    oi.getOperatorController().a().onTrue(swerve.resetGyroCommand());
    oi.getOperatorController().x().onTrue(manipulator.turnOfSpeed());

    climb.setDefaultCommand(
      new InstantCommand(() -> climb.setSpeed(oi.getOperatorController().getLeftY()), climb)
    );
  }

  private void setupScoreChooser() {
    // TODO I'd recommend exploring more intuitive/quick UI for operator controls, such as the button grids many teams implemented
    scoreChooser.addOption("L3 Cone", CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE));
    scoreChooser.addOption("L3 Cube", CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CUBE));
    scoreChooser.addOption("L2 Cone", CommandFactory.setRobotState(manipulator, elevator, RobotState.L2_CONE));
    scoreChooser.addOption("L2 Cube", CommandFactory.setRobotState(manipulator, elevator, RobotState.L2_CUBE));
    scoreChooser.addOption("L1 Cone", CommandFactory.setRobotState(manipulator, elevator, RobotState.L1_CONE));
    scoreChooser.addOption("L1 Cube", CommandFactory.setRobotState(manipulator, elevator, RobotState.L1_CUBE));
    SmartDashboard.putData("Score Selecter", scoreChooser);
  }

  public Command getAutonomousCommand() { return autos.getAuto(); }
}
