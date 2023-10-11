package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
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

    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null){
            return new Manipulator();
        }
        return instance;
    }

    private Manipulator() {
        spinMotor = MotorHelper.createSparkMax(kManipulator.kSpinMotorID, MotorType.kBrushless, false,kManipulator.SPIN_CURRENT_LIMIT, IdleMode.kBrake);
        positionMotor = MotorHelper.createSparkMax(kManipulator.kPositionMotorID, MotorType.kBrushless, false, kManipulator.POSITION_CURRENT_LIMIT, IdleMode.kBrake, kManipulator.kP, kManipulator.kI, kManipulator.kD, kManipulator.kF);

        pid = new PIDTuning(kManipulator.kP, kManipulator.kI, kManipulator.kD, Constants.kTuningMode);

        wristFeedforward = new ArmFeedforward(kManipulator.kS, kManipulator.kG, kManipulator.kV, kManipulator.kA); 
        absoluteEncoder = new AbsoluteEncoder(kManipulator.ENCODER_CHANNEL, kManipulator.ENCODER_ANGLE_OFFSET);
       
        setConversionFactors();

        positionMotor.getEncoder().setPosition(absoluteEncoder.getNormalizedPosition());

        targetTunable = new TunableNumber("Wrist target", RobotState.IDLE.getWristPos(), Constants.kTuningMode);
    } 
    
    public Command setPosition(double position) {
        return runOnce(
            () -> targetTunable.setDefault(position) 
        ).until(() -> getError() < 1);
    }

    public void setConversionFactors(){
        positionMotor.getEncoder().setPositionConversionFactor(360 / kManipulator.MANIPULATOR_GEAR_RATIO);
        positionMotor.getEncoder().setVelocityConversionFactor((360 / kManipulator.MANIPULATOR_GEAR_RATIO) / 60);
    }

    public double getAbsoluteError(){
        return Math.abs(absoluteEncoder.getPosition() - positionMotor.getEncoder().getPosition());
    }

    public Command runWrist(double speed) {
        return runOnce(() -> {
            if (speed != -10) {
                spinMotor.set(speed);
            }
        });
    }

    public Command reverseCurrentWrist() {
        // Double check if applied output is right shit
        return runOnce(() -> {
            spinMotor.set(spinMotor.getAppliedOutput() * -1);
        });
    }

    public Command runWristForward() {
        return runOnce(() -> {
            spinMotor.set(kManipulator.WRIST_SPEED);
        });
    }

    public Command runWristBackward() {
        return runOnce(() -> {
            spinMotor.set(kManipulator.WRIST_REVERSE_SPEED);
        });
    }

    public Command stopWristBackward() {
        return runOnce(() -> {
            spinMotor.set(kManipulator.WRIST_STOP_SPEED);
        });
    }

    public double getError() {
        return Math.abs(positionMotor.getEncoder().getPosition() - targetTunable.get());
    }

    public double getWristPos() {
        return positionMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position", positionMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Manipulator Current", spinMotor.getOutputCurrent());

        pid.updatePID(positionMotor);

        if (getAbsoluteError() > kManipulator.ERROR_LIMIT) {
            positionMotor.getEncoder().setPosition(absoluteEncoder.getNormalizedPosition());
        }

        positionMotor.getPIDController().setReference(
            (targetTunable.get() > kManipulator.TUNE_HIGH_VAL || targetTunable.get() < kManipulator.TUNE_LOW_VAL) ? kManipulator.REFERENCE_VAL: targetTunable.get(), 
            ControlType.kPosition, 
            kManipulator.PID_SLOT, 
            wristFeedforward.calculate(Math.toRadians(positionMotor.getEncoder().getPosition()), kManipulator.WRIST_FEED_FORWARD_VELOCITY, kManipulator.WRIST_FEED_FORWARD_ACCEL)
        );
    }
}