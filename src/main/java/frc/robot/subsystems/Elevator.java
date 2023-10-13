package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.kElevator;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftElevator;
    private final CANSparkMax rightElevator;

    private PIDTuning pid;

    private final TunableNumber setpoint;
    private final ElevatorFeedforward ff;

    private static Elevator instance;

    private boolean resetElevator;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {
        ff = new ElevatorFeedforward(0, kElevator.kG, 0);

        leftElevator = MotorHelper.createSparkMax(kElevator.LEFT_MOTOR_ID, MotorType.kBrushless, false, kElevator.CURRENT_LIMIT, IdleMode.kBrake);
        rightElevator = MotorHelper.createSparkMax(kElevator.RIGHT_MOTOR_ID, MotorType.kBrushless, true, kElevator.CURRENT_LIMIT, IdleMode.kBrake, Constants.kElevator.kP, Constants.kElevator.kI, Constants.kElevator.kD, 0.0);

        leftElevator.follow(rightElevator, true);

        resetElevator = false;

        // Setup Motion Majic, this is used to reduce jerk in the elevator ???
        // rightElevator.getPIDController().setSmartMotionMaxVelocity(100, 0); // Velocity is in RPM
        // rightElevator.getPIDController().setSmartMotionMaxAccel(10, 0); // Acel in RPM^2

        if (Constants.kTuningMode) {
            pid = new PIDTuning("Elevator", kElevator.kP, kElevator.kI, kElevator.kD, Constants.kTuningMode);
        }
      
        setpoint = new TunableNumber("Elavator Setpoint", kElevator.DEFUALT_VAL, Constants.kTuningMode);
    }

    public Command moveElevator(RobotState state) {
        return run(
            () -> setpoint.setDefault(state.elevatorPos)
        ).until(() -> getError(state.elevatorPos) < 5);
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

    public double getPose() {
        return rightElevator.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Eleavator Current", rightElevator.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Position", rightElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", setpoint.get());

        if (Constants.kTuningMode) {
            pid.updatePID(rightElevator);
        }


        if (!resetElevator) {
          rightElevator.getPIDController().setReference(
            setpoint.get() > kElevator.MAX_POS || setpoint.get() < kElevator.MIN_POS ? kElevator.DEFUALT_VAL : setpoint.get(),
            CANSparkMax.ControlType.kPosition,
            0,
            ff.calculate(0.0)
          );
        }
    }
}
