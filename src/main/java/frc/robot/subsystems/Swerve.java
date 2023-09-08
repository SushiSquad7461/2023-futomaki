package frc.robot.subsytems;

import SushiFrcLib.DependencyInjection.RobotName;
import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.Sensors.gyro.Navx;
import SushiFrcLib.Sensors.gyro.Pigeon;
import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;
import frc.robot.util.SwerveModulePosition;
import frc.robot.util.SwerveModuleState;
import frc.robot.util.SwerveOdom;
import frc.robot.util.Vector;
import frc.robot.util.SwerveModule.SwerveModule;
import frc.robot.util.SwerveModule.SwerveModuleFalcon;
import frc.robot.util.SwerveModule.SwerveModuleNeo;

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
        // gyro = new Navx();
        gyro.zeroGyro();

        switch (Constants.NAME) {
            case NEO_SWERVE:
                swerveMods = new SwerveModuleNeo[]{
                    new SwerveModuleNeo(0, kSwerve.kNeoSwerve.MOD_0_Constants),
                    new SwerveModuleNeo(1, kSwerve.kNeoSwerve.MOD_1_Constants),
                    new SwerveModuleNeo(2, kSwerve.kNeoSwerve.MOD_2_Constants),
                    new SwerveModuleNeo(3, kSwerve.kNeoSwerve.MOD_3_Constants),
                };    
                break;
            default:
                swerveMods = new SwerveModuleFalcon[]{
                    new SwerveModuleFalcon(0, kSwerve.Mod0.CONSTANTS),
                    new SwerveModuleFalcon(1, kSwerve.Mod1.CONSTANTS),
                    new SwerveModuleFalcon(2, kSwerve.Mod2.CONSTANTS),
                    new SwerveModuleFalcon(3, kSwerve.Mod3.CONSTANTS),
                }; 
                break;
        }

        odom = new SwerveOdom(kSwerve.SWERVE_KINEMATICS);

        field = new Field2d();
        SmartDashboard.putData("Field", field);

    }

    // Vector is in mps, and rot is in radians per sec
    public void drive(Vector vector, double rot) {
        SmartDashboard.putString("Input: ", vector.x + ", " + vector.y + ", " + rot);


        vector.rotate(gyro.getAngle());
 
        SmartDashboard.putString("Input Post Rotate : ", vector.x + ", " + vector.y + ", " + rot);

        SwerveModuleState[] states = kSwerve.SWERVE_KINEMATICS.getStates(vector, rot);

        // TODO: FIX SHITY CODE https://github.com/frc1678/C2023-Public/blob/main/src/main/java/com/team1678/lib/swerve/SwerveDriveKinematics.java
        for (SwerveModuleState i : states) {
            if (i.velocity > kSwerve.MAX_SPEED) {
                i.velocity = kSwerve.MAX_SPEED;
            }
        }

        for (SwerveModule i : swerveMods) {
            SmartDashboard.putString("Swerve Module State " + i.moduleNumber, states[i.moduleNumber].velocity + ", " + states[i.moduleNumber].angle.getDegrees());
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

    @Override
    public void periodic() { 
        odom.updatePoseWithGyro(getPose(),  gyro.getAngle());

        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getAngle().getDegrees(), 0, 360));

        field.setRobotPose(odom.getPose().getPose2d());

        for (SwerveModule i : swerveMods) {
            SmartDashboard.putNumber("Swerve Module Angle " + i.moduleNumber, i.getAngle());
        }
    }
}
