package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.kManipulator;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax spinMotor;
    private final CANSparkMax positionMotor;

    private PIDTuning pid;

    private final ArmFeedforward wristFeedforwardUp;
    private final ArmFeedforward wristFeedforwardDown;
    private boolean movingUp;

    private final AbsoluteEncoder absoluteEncoder;

    private final TunableNumber targetTunable;
    // TODO what is this variable name
    private double manuSpeed;

    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            return new Manipulator();
        }
        return instance;
    }

    private Manipulator() {
        spinMotor = MotorHelper.createSparkMax(kManipulator.SPIN_MOTOR_ID, MotorType.kBrushless, false, kManipulator.SPIN_CURRENT_LIMIT, IdleMode.kBrake);
        positionMotor = MotorHelper.createSparkMax(kManipulator.POSITION_MOTOR_ID, MotorType.kBrushless, false, kManipulator.POSITION_CURRENT_LIMIT, IdleMode.kBrake, kManipulator.P_UP, kManipulator.I, kManipulator.D, 0.0);

        if (Constants.TUNING_MODE) {
            pid = new PIDTuning("Maniupaltor", kManipulator.P_UP, kManipulator.I, kManipulator.D, Constants.TUNING_MODE);
        }

        wristFeedforwardUp = new ArmFeedforward(0.0, kManipulator.G_UP, 0.0); 
        wristFeedforwardDown = new ArmFeedforward(0.0, kManipulator.G_DOWN, 0.0); 

        absoluteEncoder = new AbsoluteEncoder(kManipulator.ENCODER_CHANNEL, kManipulator.ENCODER_ANGLE_OFFSET);
       
        positionMotor.getEncoder().setPositionConversionFactor(360 / kManipulator.MANIPULATOR_GEAR_RATIO);
        positionMotor.getEncoder().setVelocityConversionFactor((360 / kManipulator.MANIPULATOR_GEAR_RATIO) / 60);

        targetTunable = new TunableNumber("Wrist target", kManipulator.DEFUALT_VAL, Constants.TUNING_MODE);
        manuSpeed = 0.0;

        movingUp = true;
    } 
    
    public Command setPosition(RobotState state) {
        return new SequentialCommandGroup(
            runOnce(
                () ->  {
                    movingUp = state.wristPos > positionMotor.getEncoder().getPosition();
                    positionMotor.getPIDController().setP(movingUp ? kManipulator.P_UP : kManipulator.P_DOWN); // setting the P
                    targetTunable.setDefault(state.wristPos);
                }
            ),
            new WaitUntilCommand(closeToSetpoint(state.wristPos))
        );
    }

    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (getError(setpoint) < kManipulator.MAX_ERROR);
    }

    public double getAbsoluteError(){
        return Math.abs(getWristPos() - absoluteEncoder.getNormalizedPosition());
    }

    public Command runWrist(RobotState state) {
        return runOnce(() -> {
            manuSpeed = state.manipulatorSpeed;
         });
    }

    public Command reverseCurrentWrist() {
        return runOnce(() -> manuSpeed = manuSpeed * -1);
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

    public Command turnOfSpeed() {
        return runOnce(() -> manuSpeed = 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", positionMotor.getEncoder().getPosition());

        if(Constants.TUNING_MODE) {
            pid.updatePID(positionMotor);
        }

        if (getAbsoluteError() > kManipulator.ERROR_LIMIT) {
            resetWristPos();
        }

        spinMotor.set(manuSpeed);

        positionMotor.getPIDController().setReference(
            targetTunable.get(),
            ControlType.kPosition, 
            0,
            (movingUp ? wristFeedforwardUp : wristFeedforwardDown).calculate(Math.toRadians(positionMotor.getEncoder().getPosition()), 0.0)
        );
    }
}