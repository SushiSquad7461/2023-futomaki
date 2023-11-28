package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;

public class Elevator extends SubsystemBase {
    private PIDTuning pid;
    private final TunableNumber setpoint;
    private final ElevatorIO io;

    private static Elevator instance;
    private boolean resetElevator;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {
        io = new ElevatorIOReal();
        resetElevator = false;

        if (Constants.TUNING_MODE) {
            pid = new PIDTuning("Elevator", Constants.Elevator.RIGHT_MOTOR.pid, Constants.TUNING_MODE);
        }
      
        setpoint = new TunableNumber("Elavator Setpoint", Constants.DEFUAL_STATE.elevatorPos, Constants.TUNING_MODE);
    }

    public Command moveElevator(RobotState state) {
        return new SequentialCommandGroup(
            runOnce(
                () -> {
                    setpoint.setDefault(state.elevatorPos);
                }
            ),
            new WaitUntilCommand(io.closeToSetpoint(state.elevatorPos))
        );
    }

    // TODO I'd recommend making reset sequences a bit more fool-proof. Too easy to make a mistake and call reset start without calling reset end
    public Command resetElevatorPoseStart() {
        return runOnce(
            () -> {
                io.applySpeed(-0.1);
                resetElevator = true;
            }
        );
    }

    public Command resetElevatorPoseEnd() {
        return runOnce(
            () -> {
                io.applySpeed(0.0);
                io.setPosition(0.0);
                resetElevator = false;
            }
        );
    }

    public double getPose() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Elevator/Position", getPose());

        if (Constants.TUNING_MODE) {
            io.updatePID(pid);
        }

        if (!resetElevator) {
            io.setSetpoint(setpoint.get());
        }
    }
}
