package frc.robot.util;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Swerve.SDSModules;

public class FutomakiSwerveModule extends SwerveModuleConstants {

    public FutomakiSwerveModule(int moduleNumber, double angleOffset, SDSModules moduleInfo) {
        super(moduleNumber, angleOffset, moduleInfo, kSwerve.MAX_SPEED, kSwerve.SWERVE_TUNNING_MODE);
    }

    @Override
    public CANSparkMax getDriveNeo() {
        CANSparkMax driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(kSwerve.DRIVE_INVERSION);
        driveMotor.setIdleMode(driveIdleMode);
        driveMotor.setSmartCurrentLimit(kSwerve.DRIVE_CURRENT_LIMIT);

        SparkMaxPIDController drivePID = driveMotor.getPIDController();
        drivePID.setP(kSwerve.DRIVE_P);
        drivePID.setI(kSwerve.DRIVE_I);
        drivePID.setD(kSwerve.DRIVE_D);
        drivePID.setFF(kSwerve.DRIVE_F);

        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(driveRotationsToMeters);
        driveEncoder.setVelocityConversionFactor(driveRMPToMetersPerSec);
        driveEncoder.setPosition(0);

        return driveMotor;
    }

    @Override
    public WPI_TalonFX getDriveFalcon() {
        return MotorHelper.createFalconMotor(driveMotorId, 
            kSwerve.DRIVE_CURRENT_LIMIT,
            kSwerve.DRIVE_INVERSION, driveNeutralMode, 
            kSwerve.DRIVE_P, kSwerve.DRIVE_I, kSwerve.DRIVE_D,
            kSwerve.DRIVE_F,
            kPorts.CANIVORE_NAME, SensorInitializationStrategy.BootToZero
        );
    }

    @Override
    public CANSparkMax getAngleNeo() {
        CANSparkMax neo = new CANSparkMax(angleMotorId, MotorType.kBrushless);

        neo.restoreFactoryDefaults();
        neo.setInverted(kSwerve.ANGLE_INVERSION);
        neo.setIdleMode(angleIdleMode);
        neo.setSmartCurrentLimit(kSwerve.ANGLE_CURRENT_LIMIT);

        SparkMaxPIDController anglePID = neo.getPIDController();

        anglePID.setP(kSwerve.ANGLE_P);
        anglePID.setI(kSwerve.ANGLE_I);
        anglePID.setD(kSwerve.ANGLE_D);
        anglePID.setFF(kSwerve.ANGLE_F);

        RelativeEncoder angleEncoder = neo.getEncoder();
        angleEncoder.setPositionConversionFactor(angleRotationsToDegrees);
        angleEncoder.setVelocityConversionFactor(angleRMPToDegreesPerSec);

        return neo;
    }

    @Override
    public WPI_TalonFX getAngleFalcon() {
        return MotorHelper.createFalconMotor(angleMotorId, 
            kSwerve.ANGLE_CURRENT_LIMIT,
            kSwerve.ANGLE_INVERSION, angleNeutralMode, 
            kSwerve.ANGLE_P, kSwerve.ANGLE_I, kSwerve.ANGLE_D,
            kSwerve.ANGLE_F,
            kPorts.CANIVORE_NAME, SensorInitializationStrategy.BootToZero
        );
    }

    @Override
    public WPI_CANCoder getCanCoder() {
        WPI_CANCoder angleEncoder = new WPI_CANCoder(cancoderId, kPorts.CANIVORE_NAME);

        angleEncoder.configFactoryDefault();
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleEncoder.configSensorDirection(kSwerve.CANCODER_INVERSION);
        angleEncoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );

        return angleEncoder;
    }
}
