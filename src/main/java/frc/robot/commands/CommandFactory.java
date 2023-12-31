package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

// TODO can swap to functional composition for constructing these commands, just looks a little cleaner
public class CommandFactory {
    public static Command setRobotState(Manipulator manipulator, Elevator elevator, RobotState state) {
        // TODO I'd avoid this pattern if possible because it basically makes the command un-cancellable. Not a big deal with how you are using it but still
        return new InstantCommand(() -> {
            boolean moveElevatorFirst =  state.elevatorPos > elevator.getPose() && state.wristPos < manipulator.getWristPos();

            new SequentialCommandGroup(
                moveElevatorFirst ? elevator.moveElevator(state) : manipulator.setPosition(state),
                moveElevatorFirst ? manipulator.setPosition(state) : elevator.moveElevator(state),
                state.changeSpeed ? manipulator.runWrist(state) : new InstantCommand()
            ).schedule();  
            }, manipulator, elevator);
    }

    public static Command setRobotStateElevatorFirst(Manipulator manipulator, Elevator elevator, RobotState state) {
        return new SequentialCommandGroup(
                state.changeSpeed ? manipulator.runWrist(state) : new InstantCommand(),
                elevator.moveElevator(state),
                manipulator.setPosition(state)
            );
    }

    public static Command setRobotStateWristFirst(Manipulator manipulator, Elevator elevator, RobotState state) {
        return new SequentialCommandGroup(
            manipulator.setPosition(state),
            elevator.moveElevator(state),
            state.changeSpeed ? manipulator.runWrist(state) : new InstantCommand()
        );
    }
}
