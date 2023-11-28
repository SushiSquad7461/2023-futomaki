package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator.Elevator;

public class CommandFactory {
    public static Command setRobotState(Manipulator manipulator, Elevator elevator, RobotState state) {
        return new ConditionalCommand(
            setRobotStateElevatorFirst(manipulator, elevator, state), 
            setRobotStateWristFirst(manipulator, elevator, state), 
            () -> (state.elevatorPos > elevator.getPose() && state.wristPos < manipulator.getWristPos())
        );
    }

    public static Command setRobotStateElevatorFirst(Manipulator manipulator, Elevator elevator, RobotState state) {
        return manipulator.runWrist(state).
                andThen(elevator.moveElevator(state)).
                andThen(manipulator.setPosition(state));
    }

    public static Command setRobotStateWristFirst(Manipulator manipulator, Elevator elevator, RobotState state) {
        return manipulator.setPosition(state).
            andThen(elevator.moveElevator(state)).
            andThen(manipulator.runWrist(state));
    }
}
