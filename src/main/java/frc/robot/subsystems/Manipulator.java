package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kManipulator;

public class Manipulator extends SubsystemBase {
    CANSparkMax spinMotor;
    CANSparkMax positionMotor;
    TunableNumber kP;
    TunableNumber kI;
    TunableNumber kD;
    TunableNumber setpoint;
    ArmFeedforward armFeedforward;
    SparkMaxPIDController positionPIDControl;


    public Manipulator() {
        spinMotor = MotorHelper.createSparkMax(kManipulator.kSpinMotorID, MotorType.kBrushless);
        spinMotor.setIdleMode(IdleMode.kBrake);

        positionMotor = new CANSparkMax(kManipulator.kPositionMotorID, MotorType.kBrushless);
        positionMotor.setIdleMode(IdleMode.kBrake);
        positionPIDControl = positionMotor.getPIDController();

        kP = new TunableNumber("wrist kP", kManipulator.kP, Constants.kTuningMode);
        kI = new TunableNumber("wrist kI", kManipulator.kI, Constants.kTuningMode);
        kD = new TunableNumber("wrist kD", kManipulator.kD, Constants.kTuningMode);
        setpoint = new TunableNumber("wrist Setpoint", 0, Constants.kTuningMode);
        armFeedforward = new ArmFeedforward(kManipulator.kS, kManipulator.kG, kManipulator.kV);

        positionPIDControl.setP(kP.get());
        positionPIDControl.setI(kI.get());
        positionPIDControl.setD(kD.get());
    }

    @Override
    public void periodic() {
        if (kP.hasChanged()) positionPIDControl.setP(kP.get());
        if (kI.hasChanged()) positionPIDControl.setI(kI.get());
        if (kD.hasChanged()) positionPIDControl.setD(kD.get());

        SmartDashboard.putNumber("Motor Position", positionMotor.getEncoder().getPosition());

        positionPIDControl.setReference(
                setpoint.get(), 
                com.revrobotics.CANSparkMax.ControlType.kPosition, 
                0, 
                armFeedforward.calculate(positionMotor.getEncoder().getPosition(), 0)
            );
    }

    public Command moveArm(double newPos) {
        return runOnce(() -> setpoint.setDefault(newPos));
    }

    public Command spin(double speed) {
        return runOnce(() -> spinMotor.set(speed));
    }
}