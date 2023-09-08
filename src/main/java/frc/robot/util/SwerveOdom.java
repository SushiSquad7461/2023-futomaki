package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;

public class SwerveOdom {
    private RobotPose currPos;
    private SwerveKinematics kinematics;
    private double oldTime;

    private SwerveModulePosition[] prevSwerveModulePositions;

    public SwerveOdom(SwerveKinematics kinematics) {
        this.kinematics = kinematics;

        currPos = new RobotPose(new Vector(0, 0), new Rotation2d(0));

        prevSwerveModulePositions = new SwerveModulePosition[]{null, null, null, null};

        for (int i=0; i < 4; ++i) {
            prevSwerveModulePositions[i] = new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
        }

        oldTime = Timer.getFPGATimestamp();
    }

    public void setPose(RobotPose newPos) {
        currPos = newPos;
    }

    public void updatePoseWithGyro(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroAngle) {
        Twist2d twist = kinematics.getTwistFromDeltra(getSwerveModuleDelta(swerveModulePositions));

        twist.dtheta = gyroAngle.minus(currPos.rot).getRadians();

        Pose2d pose = (new Pose2d(new Translation2d(currPos.loc.x, currPos.loc.y), currPos.rot)).exp(twist);

        currPos = new RobotPose(new Vector(pose.getX(), pose.getY()), gyroAngle);
    }

    private SwerveModulePosition[] getSwerveModuleDelta(SwerveModulePosition[] swerveModulePositions) {
        SwerveModulePosition[] states = new SwerveModulePosition[]{null, null, null, null};

        for (int i=0; i < 4; ++i) {
            states[i] = new SwerveModulePosition(
                (swerveModulePositions[i].distance - prevSwerveModulePositions[i].distance), 
                swerveModulePositions[i].angle
            );
        }

        prevSwerveModulePositions = swerveModulePositions;
        oldTime = Timer.getFPGATimestamp();

        return states;
    }

    public RobotPose getPose() {
        return currPos;
    }
}
