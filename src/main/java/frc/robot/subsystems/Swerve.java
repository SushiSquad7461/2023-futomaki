package frc.robot.subsystems;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.Sensors.gyro.Pigeon;
import SushiFrcLib.SmartDashboard.AllianceColor;
import SushiFrcLib.Swerve.SwerveOdom;
import SushiFrcLib.Swerve.SwerveModules.SwerveModule;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleNeoFalcon;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;
import frc.robot.util.BaseSwerve;

public class Swerve extends BaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private PIDController locationLockPID;

    private AllianceColor color;

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

        color = AllianceColor.getInstance();
    }

    public void turnOnLocationLock(double angle) {
        locationLock = true;

        if (color.isRed()) {
            angle += 180;
        }
        
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
