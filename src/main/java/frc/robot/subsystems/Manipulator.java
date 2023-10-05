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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.kManipulator;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax spinMotor;
    private final CANSparkMax positionMotor;
    private final PIDTuning pid;


    private final ArmFeedforward wristFeedforward;
    private final AbsoluteEncoder absoluteEncoder;

     private final TunableNumber targetTunable;

    public static Manipulator getInstance() {
        return new Manipulator();
    }

    // 1 8 for upright cone

    private Manipulator() {
        spinMotor = MotorHelper.createSparkMax(kManipulator.kSpinMotorID, MotorType.kBrushless);
        spinMotor.setSmartCurrentLimit(40);
        positionMotor = new CANSparkMax(kManipulator.kPositionMotorID, MotorType.kBrushless); // use MotorHelper.createSpark max

        pid = new PIDTuning(kManipulator.kP, kManipulator.kI, kManipulator.kD, Constants.kTuningMode);
        wristFeedforward = new ArmFeedforward(kManipulator.kS, kManipulator.kG, kManipulator.kV, kManipulator.kA); 

        absoluteEncoder = new AbsoluteEncoder(kManipulator.Encoder_Channel, kManipulator.ENCODER_ANGLE_OFFSET);
       
        positionMotor.getEncoder().setPositionConversionFactor(360 / kManipulator.ManipulatorGearRatio);
        positionMotor.getEncoder().setVelocityConversionFactor((360 / kManipulator.ManipulatorGearRatio) / 60);

        positionMotor.getEncoder().setPosition(absoluteEncoder.getNormalizedPosition());

        targetTunable = new TunableNumber("Wrist target", RobotState.IDLE.getWristPos(), Constants.kTuningMode);
    } 
    
    public Command setPosition(double position) {
        return runOnce(
            () -> targetTunable.setDefault(position) 
        ).until(() -> getError() < 1);
    }

    public Command runWrist(double speed) {
        return runOnce(() -> spinMotor.set(speed));
    }

    public Command runWristForward() {
        return runOnce(() -> {
            spinMotor.set(1.0);
        });
    }

    public Command runWirstBackward() {
        return runOnce(() -> {
            spinMotor.set(-1.0);
        });
    }

    public Command stopWirstBackward() {
        return runOnce(() -> {
            spinMotor.set(0.0);
        });
    }

    public double getError() {
        return Math.abs(positionMotor.getEncoder().getPosition() - targetTunable.get());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Positon", positionMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Absolute Positon", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Manipulator Current", spinMotor.getOutputCurrent());

        pid.updatePID(positionMotor);

        if (Math.abs(absoluteEncoder.getPosition() - positionMotor.getEncoder().getPosition()) > 1 ) {
            positionMotor.getEncoder().setPosition(absoluteEncoder.getNormalizedPosition());
        }

        positionMotor.getPIDController().setReference(
            targetTunable.get() > 100 || targetTunable.get() < -30 ? 0 : targetTunable.get(), 
            ControlType.kPosition, 
            0, 
            wristFeedforward.calculate(Math.toRadians(positionMotor.getEncoder().getPosition()), 0, 0)
        );
    }
}