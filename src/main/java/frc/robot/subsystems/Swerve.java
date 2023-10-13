package frc.robot.subsystems;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.Sensors.gyro.Pigeon;
import SushiFrcLib.Swerve.swerveModules.SwerveModule;
import SushiFrcLib.Swerve.swerveModules.SwerveModuleNeoFalcon;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;
import frc.robot.util.SwerveOdom;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final Gyro gyro;
    private final SwerveOdom odom;
    private final Field2d field;

    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }



    private Swerve() {
        gyro = new Pigeon(kPorts.PIGEON_ID, kSwerve.GYRO_INVERSION, kPorts.CANIVORE_NAME);
        gyro.zeroGyro();


        swerveMods = new SwerveModuleNeoFalcon[]{
            new SwerveModuleNeoFalcon(kSwerve.MOD0_CONSTANTS),
            new SwerveModuleNeoFalcon(kSwerve.MOD1_CONSTANTS),
            new SwerveModuleNeoFalcon(kSwerve.MOD2_CONSTANTS),
            new SwerveModuleNeoFalcon(kSwerve.MOD3_CONSTANTS),
        }; 

        odom = new SwerveOdom(kSwerve.SWERVE_KINEMATICS, getPose());

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond), chassisSpeeds.omegaRadiansPerSecond);
    }

    // Vector is in mps, and rot is in radians per sec
    public void drive(Translation2d vector, double rot) {
        if (kSwerve.SWERVE_TUNNING_MODE) {
            SmartDashboard.putString("Input: ", vector.getX() + ", " + vector.getY() + ", " + rot);
        }

        vector = vector.rotateBy(gyro.getAngle());

        if (kSwerve.SWERVE_TUNNING_MODE) {
            SmartDashboard.putString("Input Post Rotate : ", vector.getX() + ", " + vector.getY() + ", " + rot);
        }

        SwerveModuleState[] states = kSwerve.SWERVE_KINEMATICS.getStates(vector, rot);

        // TODO: FIX SHITY CODE https://github.com/frc1678/C2023-Public/blob/main/src/main/java/com/team1678/lib/swerve/SwerveDriveKinematics.java
        for (SwerveModuleState i : states) {
            if (i.speedMetersPerSecond > kSwerve.MAX_SPEED) {
                i.speedMetersPerSecond = kSwerve.MAX_SPEED;
            }
        }

        for (SwerveModule i : swerveMods) {
            if (kSwerve.SWERVE_TUNNING_MODE) {
                SmartDashboard.putString("Swerve Module State " + i.moduleNumber, states[i.moduleNumber].speedMetersPerSecond + ", " + states[i.moduleNumber].angle.getDegrees());
            }
            i.setDesiredState(states[i.moduleNumber]);
        }
    }

    public void updateEncoders() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModulePosition[] getPose() {
        SwerveModulePosition[] ret = new SwerveModulePosition[]{null, null, null, null};

        for (SwerveModule i : swerveMods) {
            ret[i.moduleNumber] = i.getPose();
        } 

        return ret;
    }

    public void setOdomPose(Pose2d pose) { odom.setPose(pose); }

    public Pose2d getOdomPose() { return odom.getPose(); }

    public Gyro getGyro() { return gyro; }

    public void resetGyro() {
        gyro.zeroGyro();
    }

    @Override
    public void periodic() { 
        odom.updatePoseWithGyro(getPose(),  gyro.getAngle());

        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getAngle().getDegrees(), 0, 360));

        field.setRobotPose(odom.getPose());

        for (SwerveModule i : swerveMods) {
            if (kSwerve.SWERVE_TUNNING_MODE) {
                SmartDashboard.putNumber("Swerve Module Angle " + i.moduleNumber, i.getAngle());
            }
            i.log();
        }
    }
}
