package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class SetRobotState extends CommandBase {
    private Manipulator manipulator;
    private Elevator elevator;
    private RobotState state;

    public SetRobotState(Manipulator manipulator, Elevator elevator, RobotState state) {
        this.manipulator = manipulator;
        this.elevator = elevator;
        this.state = state;
    }

    @Override
    public void initialize() {
        super.initialize();
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
}
