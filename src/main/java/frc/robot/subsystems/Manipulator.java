package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kManipulator;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax spinMotor;
    private final CANSparkMax positionMotor;

    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;

    private final ArmFeedforward wristFeedforward;
    private final SparkMaxPIDController positionPIDControl;
    private final AbsoluteEncoder absoluteEncoder;

    private double target;

    public Manipulator() {
        spinMotor = MotorHelper.createSparkMax(kManipulator.kSpinMotorID, MotorType.kBrushless);
        positionMotor = MotorHelper.createSparkMax(kManipulator.kPositionMotorID, MotorType.kBrushless);
        positionPIDControl = positionMotor.getPIDController();

        kP = new TunableNumber("kP", kManipulator.kP, Constants.kTuningMode);
        kI = new TunableNumber("kI", kManipulator.kI, Constants.kTuningMode);
        kD = new TunableNumber("kD", kManipulator.kD, Constants.kTuningMode);

        wristFeedforward = new ArmFeedforward(kManipulator.kS, kManipulator.kG, kManipulator.kV, kManipulator.kA); 
        absoluteEncoder = new AbsoluteEncoder(kManipulator.Encoder_Channel, kManipulator.ENCODER_ANGLE_OFFSET);
        

        positionMotor.getEncoder().setPositionConversionFactor(360/ kManipulator.GearRatio);
        positionMotor.getEncoder().setVelocityConversionFactor(360/ kManipulator.GearRatio / 60);

        positionMotor.getEncoder().setPosition(absoluteEncoder.getPosition()); //setting relative to absolute encoder

    } 
    
    public void setAngle(double speed) {
        target = speed;
    }

    public void setPosition(double position) {
        positionMotor.getEncoder().setPosition(position);
    }

    @Override
    public void periodic() {
        if (kP.hasChanged()) positionPIDControl.setP(kP.get());
        if (kD.hasChanged()) positionPIDControl.setD(kD.get());

        positionPIDControl.setReference(
            target, 
            ControlType.kPosition, 
            0, 
            wristFeedforward.calculate((positionMotor.getEncoder().getPosition()) * Math.PI/180, 0, 0)
        );
    }
}