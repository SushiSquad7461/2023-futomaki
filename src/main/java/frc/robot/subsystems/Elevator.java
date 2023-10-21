package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
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

    private final DigitalInput hello = new DigitalInput(9);

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {
        ffd = new ElevatorFeedforward(0, kElevator.kG_DOWN, 0);
        ffu = new ElevatorFeedforward(0, kElevator.kG_UP, 0);
        up = true;

        leftElevator = MotorHelper.createSparkMax(kElevator.LEFT_MOTOR_ID, MotorType.kBrushless, false, kElevator.CURRENT_LIMIT, IdleMode.kBrake);
        rightElevator = MotorHelper.createSparkMax(kElevator.RIGHT_MOTOR_ID, MotorType.kBrushless, true, kElevator.CURRENT_LIMIT, IdleMode.kBrake, Constants.kElevator.kP_UP, Constants.kElevator.kI, Constants.kElevator.kD, 0.0);

        leftElevator.follow(rightElevator, true);

        resetElevator = false;

        // Setup Motion Majic, this is used to reduce jerk in the elevator ???
        // rightElevator.getPIDController().setSmartMotionMaxVelocity(100, 0); // Velocity is in RPM
        // rightElevator.getPIDController().setSmartMotionMaxAccel(10, 0); // Acel in RPM^2

        if (Constants.kTuningMode) {
            pid = new PIDTuning("Elevator", kElevator.kP_UP, kElevator.kI, kElevator.kD, Constants.kTuningMode);
        }
      
        setpoint = new TunableNumber("Elavator Setpoint", kElevator.DEFUALT_VAL, Constants.kTuningMode);
    }

    public Command moveElevator(RobotState state) {
        return new SequentialCommandGroup(
            run(
                () -> {
                    if (state.elevatorPos > getPose()) {
                        up = true;
                        rightElevator.getPIDController().setP(Constants.kElevator.kP_UP);
                    } else {
                        up = false;
                        rightElevator.getPIDController().setP(Constants.kElevator.kP_DOWN);
                    }

                    setpoint.setDefault(state.elevatorPos);
                }
            ),
            new WaitUntilCommand(closeToSetpoint(state.elevatorPos))
        );
    }

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
        return () -> (getError(setpoint) < 5);
    }

    public double getPose() {
        return rightElevator.getEncoder().getPosition();
    }



    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Eleavator Current", rightElevator.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Position", rightElevator.getEncoder().getPosition());
        // SmartDashboard.putNumber("Elevator Setpoint", setpoint.get());
        SmartDashboard.putNumber("Elebator error", getError(setpoint.get()));

        SmartDashboard.putBoolean("Hrllo ", hello.get());

        if (Constants.kTuningMode) {
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
