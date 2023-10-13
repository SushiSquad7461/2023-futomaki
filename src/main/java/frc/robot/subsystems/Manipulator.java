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

    private PIDTuning pid;

    private final ArmFeedforward wristFeedforward;
    private final AbsoluteEncoder absoluteEncoder;

    private final TunableNumber targetTunable;
    private final TunableNumber manuSpeed;

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

        if (Constants.kTuningMode) {
            pid = new PIDTuning("Maniupaltor", kManipulator.kP, kManipulator.kI, kManipulator.kD, Constants.kTuningMode);
        }

        wristFeedforward = new ArmFeedforward(0.0, kManipulator.kG, 0.0); 
        absoluteEncoder = new AbsoluteEncoder(kManipulator.ENCODER_CHANNEL, kManipulator.ENCODER_ANGLE_OFFSET);
       
        positionMotor.getEncoder().setPositionConversionFactor(360 / kManipulator.MANIPULATOR_GEAR_RATIO);
        positionMotor.getEncoder().setVelocityConversionFactor((360 / kManipulator.MANIPULATOR_GEAR_RATIO) / 60);

        targetTunable = new TunableNumber("Wrist target", kManipulator.DEFUALT_VAL, Constants.kTuningMode);
        manuSpeed = new TunableNumber("Manu speed", 0.0, Constants.kTuningMode);
    } 
    
    public Command setPosition(RobotState state) {
        return run(
            () ->  {
                targetTunable.setDefault(state.wristPos);
            }
        ).until(() -> getError(state.wristPos) < 5);
    }

    public double getAbsoluteError(){
        return Math.abs(getWristPos() - absoluteEncoder.getNormalizedPosition());
    }

    public Command runWrist(RobotState state) {
        return runOnce(() -> {
            manuSpeed.setDefault(state.manipulatorSpeed);
         });
    }

    public Command reverseCurrentWrist() {
        // Double check if applied output is right shit
        return runOnce(() -> {
            manuSpeed.setDefault(manuSpeed.get() * -1);
        });
    }

    public double getError(double setpoint) {
        return Math.abs(getWristPos() - setpoint);
    }

    public double getWristPos() {
        return positionMotor.getEncoder().getPosition();
    }

    public void resetWristPos() {
        positionMotor.getEncoder().setPosition(absoluteEncoder.getNormalizedPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", positionMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Setpoint", targetTunable.get());
        SmartDashboard.putNumber("Absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Manipulator Current", spinMotor.getOutputCurrent());

        if(Constants.kTuningMode) {
            pid.updatePID(positionMotor);
        }

        if (getAbsoluteError() > kManipulator.ERROR_LIMIT) { // figure out why cmomenting this out is breaking
            resetWristPos();
        }

        spinMotor.set(manuSpeed.get());

        positionMotor.getPIDController().setReference(
            // (targetTunable.get() > kManipulator.TUNE_HIGH_VAL || targetTunable.get() < kManipulator.TUNE_LOW_VAL) ? kManipulator.DEFUALT_VAL: targetTunable.get(), 
            targetTunable.get(),
            ControlType.kPosition, 
            0,
            wristFeedforward.calculate(Math.toRadians(positionMotor.getEncoder().getPosition()), 0.0)
        );
    }
}