package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kElevator;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftElevator;
    private final CANSparkMax rightElavtor;

    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;
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
        leftElevator = MotorHelper.createSparkMax(3, MotorType.kBrushless, false, 40, IdleMode.kBrake);
        rightElavtor = MotorHelper.createSparkMax(20, MotorType.kBrushless, true, 40, IdleMode.kBrake, Constants.kElevator.kP, Constants.kElevator.kI, Constants.kElevator.kD, 0);

        leftElevator.follow(rightElavtor, true);

        kP = new TunableNumber("kP", Constants.kElevator.kP, Constants.kTuningMode);
        kI = new TunableNumber("kI", Constants.kElevator.kI, Constants.kTuningMode);
        kD = new TunableNumber("kD", Constants.kElevator.kD, Constants.kTuningMode);
        setpoint = new TunableNumber("Setpoint", 0, Constants.kTuningMode);
        ff = new ElevatorFeedforward(0, kElevator.kG, 0);
    }

    public void run(double speed) {
        rightElavtor.set(speed);
    }

    public void pid(double value) {
        setpoint.setDefault(value);
        rightElavtor.getPIDController().setReference(setpoint.get(), ControlType.kPosition);
    }


    @Override
    public void periodic() {
        if (kP.hasChanged()) {
            rightElavtor.getPIDController().setP(kP.get());
            leftElevator.getPIDController().setP(kP.get());
        }
        if (kI.hasChanged()) {
            rightElavtor.getPIDController().setI(kI.get());
            leftElevator.getPIDController().setI(kI.get());
        }
        if (kD.hasChanged()) {
            rightElavtor.getPIDController().setD(kD.get());
            leftElevator.getPIDController().setD(kD.get());
        }


        rightElavtor.getPIDController().setReference(
            setpoint.get(),
            CANSparkMax.ControlType.kPosition,
            0,
            ff.calculate(0)
        );


        SmartDashboard.putNumber("Current", rightElavtor.getOutputCurrent());
        SmartDashboard.putNumber("Left Position", leftElevator.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Position", rightElavtor.getEncoder().getPosition());

    }
}
