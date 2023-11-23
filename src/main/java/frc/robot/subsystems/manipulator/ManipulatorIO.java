package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import frc.robot.Constants.kManipulator;

public class ManipulatorIO {
    private final CANSparkMax spinMotor;
    private final CANSparkMax positionMotor;

    private final AbsoluteEncoder absoluteEncoder;

    public ManipulatorIO() {
        positionMotor = MotorHelper.createSparkMax(kManipulator.kPositionMotorID, MotorType.kBrushless, false, kManipulator.POSITION_CURRENT_LIMIT, IdleMode.kBrake, kManipulator.kP_UP, kManipulator.kI, kManipulator.kD, 0.0);
        spinMotor = MotorHelper.createSparkMax(kManipulator.kSpinMotorID, MotorType.kBrushless, false,kManipulator.SPIN_CURRENT_LIMIT, IdleMode.kBrake);

        positionMotor.getEncoder().setPositionConversionFactor(360 / kManipulator.MANIPULATOR_GEAR_RATIO);
        positionMotor.getEncoder().setVelocityConversionFactor((360 / kManipulator.MANIPULATOR_GEAR_RATIO) / 60);

        absoluteEncoder = new AbsoluteEncoder(kManipulator.ENCODER_CHANNEL, kManipulator.ENCODER_ANGLE_OFFSET);
    }

    @AutoLog
   class ManipulatorIOInputs {

   } 
}
