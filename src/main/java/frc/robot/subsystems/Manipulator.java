package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kManipulator;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax spinMotor;
    private final CANSparkMax positionMotor;
    private final PIDTuning pid;

    private final ArmFeedforward wristFeedforward;
    private final AbsoluteEncoder absoluteEncoder;

    private double target;

    public Manipulator() {
        spinMotor = MotorHelper.createSparkMax(kManipulator.kSpinMotorID, MotorType.kBrushless);
        positionMotor = new CANSparkMax(kManipulator.kPositionMotorID, MotorType.kBrushless); // use MotorHelper.createSpark max

        pid = new PIDTuning(kManipulator.kP, kManipulator.kI, kManipulator.kD, Constants.kTuningMode);
        wristFeedforward = new ArmFeedforward(kManipulator.kS, kManipulator.kG, kManipulator.kV, kManipulator.kA); 

        absoluteEncoder = new AbsoluteEncoder(kManipulator.Encoder_Channel, kManipulator.ENCODER_ANGLE_OFFSET);
       
        positionMotor.getEncoder().setPositionConversionFactor(360 / kManipulator.ManipulatorGearRatio);
        positionMotor.getEncoder().setVelocityConversionFactor((360 / kManipulator.ManipulatorGearRatio) / 60);
    } 
    
    public void setAngle(double speed) {
        target = speed;
    }

    public void setPosition(double position) {
        positionMotor.getEncoder().setPosition(position);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Positon", positionMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Absolute Positon", absoluteEncoder.getPosition() );

        positionMotor.getPIDController().setReference(
            target, 
            ControlType.kPosition, 
            0, 
            wristFeedforward.calculate(positionMotor.getEncoder().getPosition(), 0, 0)
        );
    }
}