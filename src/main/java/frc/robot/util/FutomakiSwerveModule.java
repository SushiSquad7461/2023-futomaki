package frc.robot.util;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Swerve.SwerveModules.SDSModules;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleConstants;

public class FutomakiSwerveModule extends SwerveModuleConstants {

    public FutomakiSwerveModule(int moduleNumber, double angleOffset, SDSModules moduleInfo) {
        super(
            moduleNumber, 
            angleOffset, 
            moduleInfo, 
            Constants.Swerve.MAX_SPEED, 
            Constants.Swerve.SWERVE_TUNNING_MODE
        );
    }

    @Override
    public WPI_TalonFX getDriveFalcon() {
        WPI_TalonFX drive = new WPI_TalonFX(driveMotorId, Constants.Ports.CANIVORE_NAME);
        Constants.Swerve.DRIVE_CONFIG.setTalonConfig(drive);
        return drive;
    }

    @Override
    public CANSparkMax getAngleNeo() {
        CANSparkMax neo = new CANSparkMax(angleMotorId, MotorType.kBrushless);
        Constants.Swerve.ANGLE_CONFIG.setCanSparkMaxConfig(neo, MotorType.kBrushless);
        MotorHelper.setDegreeConversionFactor(neo, angleGearRatio);
        return neo;
    }

    @Override
    public WPI_CANCoder getCanCoder() {
        WPI_CANCoder angleEncoder = new WPI_CANCoder(cancoderId, Constants.Ports.CANIVORE_NAME);

        angleEncoder.configFactoryDefault();
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleEncoder.configSensorDirection(Constants.Swerve.CANCODER_INVERSION);
        angleEncoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );

        return angleEncoder;
    }
}
