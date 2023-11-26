package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import SushiFrcLib.Math.MathUtil;
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

public class Manipulator extends SubsystemBase {
    private final CANSparkMax spinMotor;
    private final CANSparkMax positionMotor;

    private PIDTuning pid;

    private final ArmFeedforward wristFeedforwardUp;
    private final ArmFeedforward wristFeedforwardDown;
    private boolean movingUp;

    private final AbsoluteEncoder absoluteEncoder;

    private final TunableNumber targetTunable;
    private double spinMotorSpeed;

    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            return new Manipulator();
        }
        return instance;
    }

    private Manipulator() {
        spinMotor = Constants.Manipulator.SPIN_MOTOR.createSparkMax();
        positionMotor =  Constants.Manipulator.POSITION_MOTOR.createSparkMax();

        if (Constants.TUNING_MODE) {
            pid = new PIDTuning("Maniupaltor", Constants.Manipulator.POSITION_MOTOR.pid, Constants.TUNING_MODE);
        }

        wristFeedforwardUp = new ArmFeedforward(0.0, Constants.Manipulator.G_UP, 0.0); 
        wristFeedforwardDown = new ArmFeedforward(0.0, Constants.Manipulator.G_DOWN, 0.0); 

        absoluteEncoder = new AbsoluteEncoder(Constants.Manipulator.ENCODER_CHANNEL, Constants.Manipulator.ENCODER_ANGLE_OFFSET);

        MotorHelper.setConversionFactor(positionMotor, Constants.Manipulator.GEAR_RATIO);

        targetTunable = new TunableNumber("Wrist target",  Constants.DEFUAL_STATE.wristPos, Constants.TUNING_MODE);

        spinMotorSpeed = 0.0;
        movingUp = true;
    } 
    
    public Command setPosition(RobotState state) {
        return new SequentialCommandGroup(
            runOnce(
                () ->  {
                    movingUp = state.wristPos > positionMotor.getEncoder().getPosition();
                    positionMotor.getPIDController().setP(movingUp ? Constants.Manipulator.P_UP : Constants.Manipulator.P_DOWN); // setting the P
                    targetTunable.setDefault(state.wristPos);
                }
            ),
            new WaitUntilCommand(closeToSetpoint(state.wristPos))
        );
    }

    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (MathUtil.getError(positionMotor, setpoint) < Constants.Manipulator.MAX_ERROR);
    }

    public Command runWrist(RobotState state) {
        return runOnce(() -> {
            if (state.changeSpeed) {
                spinMotorSpeed = state.manipulatorSpeed;
            }
         });
    }

    public Command reverseCurrentWrist() {
        return runOnce(() -> spinMotorSpeed = spinMotorSpeed * -1);
    }

    public void resetWristPos() {
        positionMotor.getEncoder().setPosition(absoluteEncoder.getNormalizedPosition());
    }

    public double getWristPos() {
        return positionMotor.getEncoder().getPosition();
    }

    public Command turnOfSpeed() {
        return runOnce(() -> spinMotorSpeed = 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", positionMotor.getEncoder().getPosition());

        if(Constants.TUNING_MODE) {
            pid.updatePID(positionMotor);
        }

        if (MathUtil.getError(positionMotor, absoluteEncoder) > Constants.Manipulator.ERROR_LIMIT) {
            resetWristPos();
        }

        spinMotor.set(spinMotorSpeed);

        positionMotor.getPIDController().setReference(
            targetTunable.get(),
            ControlType.kPosition, 
            0,
            (movingUp ? wristFeedforwardUp : wristFeedforwardDown).calculate(Math.toRadians(positionMotor.getEncoder().getPosition()), 0.0)
        );
    }
}