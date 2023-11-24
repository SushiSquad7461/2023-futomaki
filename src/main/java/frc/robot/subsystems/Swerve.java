package frc.robot.subsystems;

import SushiFrcLib.Sensors.gyro.Pigeon;
import SushiFrcLib.Swerve.BaseSwerve;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleNeoFalcon;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;


public class Swerve extends BaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private PIDController rotationLockPID;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }

    private Swerve() {
        super(
            new SwerveModuleNeoFalcon[]{
                new SwerveModuleNeoFalcon(Constants.Swerve.MOD0_CONSTANTS),
                new SwerveModuleNeoFalcon(Constants.Swerve.MOD1_CONSTANTS),
                new SwerveModuleNeoFalcon(Constants.Swerve.MOD2_CONSTANTS),
                new SwerveModuleNeoFalcon(Constants.Swerve.MOD3_CONSTANTS),
            },
            new Pigeon(Constants.Ports.PIGEON_ID, Constants.Swerve.GYRO_INVERSION, Constants.Ports.CANIVORE_NAME),
            Constants.Swerve.SWERVE_KINEMATICS,
            Constants.Swerve.MAX_SPEED,
            Constants.Swerve.SWERVE_TUNNING_MODE
        );

        locationLock = false;
        rotationLockPID = Constants.Swerve.autoRotate.getPIDController(); 
    }

    public void enableRotationLock(double angle) {
        locationLock = true;

        rotationLockPID.setSetpoint(angle);
        rotationLockPID.calculate(getGyro().getAngle().getDegrees());
    }

    public void disableRotationLock() {
        locationLock = false;
    }

    @Override
    public void drive(Translation2d translation, double rotation) {
        if (locationLock) {
            rotation = rotationLockPID.calculate(getGyro().getAngle().getDegrees());
        }

        drive(translation, rotation);
    }
}
