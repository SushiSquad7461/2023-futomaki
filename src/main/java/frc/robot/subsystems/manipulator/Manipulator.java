package frc.robot.subsystems.manipulator;

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
    private PIDTuning pid;

    private final ArmFeedforward wristFeedforwardUp;
    private final ArmFeedforward wristFeedforwardDown;
    private boolean movingUp;

    private double manuSpeed;

    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            return new Manipulator();
        }
        return instance;
    }

    private Manipulator() {
        if (Constants.kTuningMode) {
            pid = new PIDTuning("Maniupaltor", kManipulator.kP_UP, kManipulator.kI, kManipulator.kD, Constants.kTuningMode);
        }

        wristFeedforwardUp = new ArmFeedforward(0.0, kManipulator.kG_UP, 0.0); 
        wristFeedforwardDown = new ArmFeedforward(0.0, kManipulator.kG_DOWN, 0.0); 

        targetTunable = new TunableNumber("Wrist target", kManipulator.DEFUALT_VAL, Constants.kTuningMode);
        manuSpeed = 0.0;

        movingUp = true;
    } 
    
    public Command setPosition(RobotState state) {
        return new SequentialCommandGroup(
            runOnce(
                () ->  {
                    movingUp = state.wristPos > positionMotor.getEncoder().getPosition();
                    positionMotor.getPIDController().setP(movingUp ? kManipulator.kP_UP : kManipulator.kP_DOWN); // setting the P
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

        if(Constants.kTuningMode) {
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