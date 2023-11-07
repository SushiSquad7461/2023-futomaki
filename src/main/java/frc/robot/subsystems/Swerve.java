package frc.robot.subsystems;

import SushiFrcLib.Sensors.gyro.Pigeon;
import SushiFrcLib.Swerve.BaseSwerve;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleNeoFalcon;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;

public class Swerve extends BaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private PIDController locationLockPID;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }

    private Swerve() {
        super(
            new SwerveModuleNeoFalcon[]{
                new SwerveModuleNeoFalcon(kSwerve.MOD0_CONSTANTS),
                new SwerveModuleNeoFalcon(kSwerve.MOD1_CONSTANTS),
                new SwerveModuleNeoFalcon(kSwerve.MOD2_CONSTANTS),
                new SwerveModuleNeoFalcon(kSwerve.MOD3_CONSTANTS),
            },
            new Pigeon(kPorts.PIGEON_ID, kSwerve.GYRO_INVERSION, kPorts.CANIVORE_NAME),
            kSwerve.SWERVE_KINEMATICS,
            kSwerve.MAX_SPEED,
            kSwerve.SWERVE_TUNNING_MODE
        );

        locationLock = false;
        locationLockPID = new PIDController(
            0.1, // sir this is not tunned pls retune like rn rnrnrnnrnrnrnrnrnrn 
            0.0, 
            0.0
        );
    }

    public void turnOnLocationLock(double angle) {
        locationLock = true;

        locationLockPID.setSetpoint(angle);
        locationLockPID.calculate(getGyro().getAngle().getDegrees());
    }

    public void turnOfLocationLock() {
        locationLock = false;
    }

    public void driveWithRotationLock(Translation2d translation, double rotation) {
        if (locationLock) {
            rotation = locationLockPID.calculate(getGyro().getAngle().getDegrees());
        }

        drive(translation, rotation);
    }

    @Override
    public void periodic() { 
        super.periodic();
    }
}
