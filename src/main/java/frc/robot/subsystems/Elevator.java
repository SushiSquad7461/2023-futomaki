package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final WPI_TalonFX leftElevator;
    private final WPI_TalonFX rightElavtor;

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {
        leftElevator = MotorHelper.createFalconMotor(3, TalonFXInvertType.Clockwise);
        rightElavtor = MotorHelper.createFalconMotor(20, TalonFXInvertType.CounterClockwise);
    }

    @Override
    public void periodic() {
        leftElevator.set(ControlMode.PercentOutput, 0.1);
        rightElavtor.set(ControlMode.PercentOutput, 0.1);
    }
}
