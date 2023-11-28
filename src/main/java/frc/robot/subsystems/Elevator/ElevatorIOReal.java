package frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;

import SushiFrcLib.Math.MathUtil;
import SushiFrcLib.SmartDashboard.PIDTuning;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants;

public class ElevatorIOReal extends ElevatorIO {
    private final CANSparkMax leftElevator;
    private final CANSparkMax rightElevator; 

    private final ElevatorFeedforward ffd;
    private final ElevatorFeedforward ffu;

    public ElevatorIOReal() {
        super();

        leftElevator = Constants.Elevator.LEFT_MOTOR.createSparkMax();
        rightElevator = Constants.Elevator.RIGHT_MOTOR.createSparkMax();

        leftElevator.follow(rightElevator, true);

        ffd = new ElevatorFeedforward(0, Constants.Elevator.G_DOWN, 0);
        ffu = new ElevatorFeedforward(0, Constants.Elevator.G_UP, 0);
    }

    @Override
    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(Constants.Elevator.MAX_POS, Constants.Elevator.MIN_POS, setpoint);

        if (setpoint > inputs.previouseSetpoint) {
            inputs.up = true;
            inputs.kP = Constants.Elevator.P_UP;
            inputs.kG = Constants.Elevator.G_UP;
            rightElevator.getPIDController().setP(Constants.Elevator.P_UP);
        } else if (setpoint < inputs.previouseSetpoint) {
            inputs.up = false;
            inputs.kP = Constants.Elevator.P_DOWN;
            inputs.kG = Constants.Elevator.G_DOWN;
            rightElevator.getPIDController().setP(Constants.Elevator.P_DOWN);
        }

        inputs.previouseSetpoint = inputs.elevatorSetpoint;
        inputs.elevatorSetpoint = setpoint;
        inputs.speedMode = false;

        rightElevator.getPIDController().setReference(
            inputs.elevatorSetpoint,
            CANSparkMax.ControlType.kPosition,
            0,
            inputs.up ? ffu.calculate(0.0) : ffd.calculate(0.0)
        );    
    }

    @Override
    public double getPosition() {
        return rightElevator.getEncoder().getPosition();
    }


    @Override
    public void updatePID(PIDTuning pid) {
        pid.updatePID(rightElevator);        
    }

    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (MathUtil.getError(rightElevator, setpoint) < Constants.Elevator.MAX_ERROR);
    }

    @Override
    public void applySpeed(double speed) {
        rightElevator.set(speed);
        inputs.speed = speed;
        inputs.speedMode = true;        
    }

    @Override
    public void setPosition(double position) {
        rightElevator.getEncoder().setPosition(0.0);
    }
}
