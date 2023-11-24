package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;

import SushiFrcLib.Math.MathUtil;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftElevator;
    private final CANSparkMax rightElevator;

    private PIDTuning pid;

    private final TunableNumber setpoint;
    private final ElevatorFeedforward ffd;
    private final ElevatorFeedforward ffu;
    private static boolean up;


    private static Elevator instance;
    private boolean resetElevator;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {
        ffd = new ElevatorFeedforward(0, Constants.Elevator.G_DOWN, 0);
        ffu = new ElevatorFeedforward(0, Constants.Elevator.G_UP, 0);
        up = true;

        leftElevator = Constants.Elevator.LEFT_MOTOR.createSparkMax();
        rightElevator = Constants.Elevator.RIGHT_MOTOR.createSparkMax();

        leftElevator.follow(rightElevator, true);

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
                    up = state.elevatorPos > getPose();
                    rightElevator.getPIDController().setP(up ? Constants.Elevator.P_UP : Constants.Elevator.P_DOWN);
                    setpoint.setDefault(state.elevatorPos);
                }
            ),
            new WaitUntilCommand(closeToSetpoint(state.elevatorPos))
        );
    }

    // TODO I'd recommend making reset sequences a bit more fool-proof. Too easy to make a mistake and call reset start without calling reset end
    public Command resetElevatorPoseStart() {
        return runOnce(
            () -> {
                rightElevator.set(-0.1);
                resetElevator = true;
            }
        );
    }

    public Command resetElevatorPoseEnd() {
        return runOnce(
            () -> {
                rightElevator.set(0.0);
                rightElevator.getEncoder().setPosition(0.0);
                resetElevator = false;
            }
        );
    }

    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (MathUtil.getError(rightElevator, setpoint) < Constants.Elevator.MAX_ERROR);
    }

    public double getPose() {
        return rightElevator.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", rightElevator.getEncoder().getPosition());

        if (Constants.TUNING_MODE) {
            pid.updatePID(rightElevator);
        }

        if (!resetElevator) {
            rightElevator.getPIDController().setReference(
                MathUtil.clamp(Constants.Elevator.MAX_POS, Constants.Elevator.MIN_POS, setpoint),
                CANSparkMax.ControlType.kPosition,
                0,
                up ? ffu.calculate(0.0) : ffd.calculate(0.0)
            );
        }
    }
}
