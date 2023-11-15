package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import SushiFrcLib.Motor.MotorHelper;
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
import frc.robot.Constants.kElevator;

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
        ffd = new ElevatorFeedforward(0, kElevator.G_DOWN, 0);
        ffu = new ElevatorFeedforward(0, kElevator.G_UP, 0);
        up = true;

        leftElevator = MotorHelper.createSparkMax(kElevator.LEFT_MOTOR_ID, MotorType.kBrushless, false, kElevator.CURRENT_LIMIT, IdleMode.kBrake);
        rightElevator = MotorHelper.createSparkMax(kElevator.RIGHT_MOTOR_ID, MotorType.kBrushless, true, kElevator.CURRENT_LIMIT, IdleMode.kBrake, kElevator.P_UP, kElevator.I, kElevator.D, 0.0);

        leftElevator.follow(rightElevator, true);

        resetElevator = false;

        if (Constants.TUNING_MODE) {
            pid = new PIDTuning("Elevator", kElevator.P_UP, kElevator.I, kElevator.D, Constants.TUNING_MODE);
        }
      
        setpoint = new TunableNumber("Elavator Setpoint", kElevator.DEFUALT_VAL, Constants.TUNING_MODE);
    }

    public Command moveElevator(RobotState state) {
        return new SequentialCommandGroup(
            runOnce(
                () -> {
                    up = state.elevatorPos > getPose();
                    rightElevator.getPIDController().setP(up ? kElevator.P_UP : kElevator.P_DOWN);
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

    public double getError(double setpoint) {
        return Math.abs(rightElevator.getEncoder().getPosition() - setpoint);
    }

    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (getError(setpoint) < kElevator.MAX_ERROR);
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
                setpoint.get() > kElevator.MAX_POS || setpoint.get() < kElevator.MIN_POS ? kElevator.DEFUALT_VAL : setpoint.get(),
                CANSparkMax.ControlType.kPosition,
                0,
                up ? ffu.calculate(0.0) : ffd.calculate(0.0)
            );
        }
    }
}
