package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class CommandFactory {
    public static Command setRobotState(Manipulator manipulator, Elevator elevator, RobotState state) {
      return new InstantCommand(() -> {
        if (state.getElevatorPos() > elevator.getPose() && state.getWristPos() < manipulator.getWristPos()) {
            new SequentialCommandGroup(
              elevator.pid(state.elevatorPos),
              manipulator.setPosition(state.wristPos),
              manipulator.runWrist(1.0)
            ).schedule();  
          } else {
            new SequentialCommandGroup(
              manipulator.setPosition(state.wristPos),
              elevator.pid(state.elevatorPos),
              manipulator.runWrist(1.0)
            ).schedule(); 
          }
        }, manipulator, elevator);
    }
}
