package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kManipulator;

public class Manipulator extends SubsystemBase {
    CANSparkMax spinMotor;
    CANSparkMax positionMotor;
    TunableNumber kP;
    TunableNumber kI;
    TunableNumber kD;
    ArmFeedforward armFeedforward;
    SparkMaxPIDController positionPIDControl;


    public Manipulator() {
        spinMotor = MotorHelper.createSparkMax(kManipulator.kSpinMotorID, MotorType.kBrushless);
        positionMotor = new CANSparkMax(kManipulator.kPositionMotorID, MotorType.kBrushless);
        positionPIDControl = positionMotor.getPIDController();

        kP = new TunableNumber("kP", kManipulator.kP, Constants.kTuningMode);
        kI = new TunableNumber("kI", kManipulator.kI, Constants.kTuningMode);
        kD = new TunableNumber("kD", kManipulator.kD, Constants.kTuningMode);
        armFeedforward = new ArmFeedforward(kManipulator.kS, kManipulator.kG, kManipulator.kV, kManipulator.kA);

        
    }

    public void spin(double speed) {
        spinMotor.set(speed);
    }

    @Override
    public void periodic() {
        if (kP.hasChanged()) positionPIDControl.setP(kP.get());
        if (kD.hasChanged()) positionPIDControl.setD(kD.get());
    }
}