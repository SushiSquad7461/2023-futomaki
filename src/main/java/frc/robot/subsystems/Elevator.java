package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import SushiFrcLib.Motor.MotorHelper;
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

    // private final PIDTuning pid;

    private final TunableNumber setpoint;
    private final ElevatorFeedforward ff;

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {
        leftElevator = MotorHelper.createSparkMax(kElevator.LEFT_MOTOR_ID, MotorType.kBrushless, false, kElevator.CURRENT_LIMIT, IdleMode.kBrake);
        rightElevator = MotorHelper.createSparkMax(kElevator.RIGHT_MOTOR_ID, MotorType.kBrushless, true, kElevator.CURRENT_LIMIT, IdleMode.kBrake, Constants.kElevator.kP, Constants.kElevator.kI, Constants.kElevator.kD, 0);

        leftElevator.follow(rightElevator, true);

        // pid = new PIDTuning(kElevator.kP, kElevator.kI, kElevator.kD, Constants.kTuningMode);
      
        setpoint = new TunableNumber("Setpoint", RobotState.IDLE.elevatorPos, Constants.kTuningMode);
        ff = new ElevatorFeedforward(0, kElevator.kG, 0);
    }

    public Command pid(double value) {
        return run(
            () -> setpoint.setDefault(value)
        ).until(() -> getError() < 1);
    }

    public double getError() {
        return Math.abs(rightElevator.getEncoder().getPosition() - setpoint.get());
    }

    public double getPose() {
        return rightElevator.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        // pid.updatePID(rightElavtor);

        rightElevator.getPIDController().setReference(
            setpoint.get() > kElevator.MAX_POS || setpoint.get() < kElevator.MIN_POS ? 10 : setpoint.get(),
            CANSparkMax.ControlType.kPosition,
            0,
            ff.calculate(0)
        );

        SmartDashboard.putNumber("Current", rightElevator.getOutputCurrent());
        SmartDashboard.putNumber("Left Position", leftElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Position", rightElevator.getEncoder().getPosition());
    }
}
