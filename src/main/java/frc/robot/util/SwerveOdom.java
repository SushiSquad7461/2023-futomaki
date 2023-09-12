package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveOdom {
    private Pose2d currPos;
    private SwerveKinematics kinematics;
 
    private SwerveModulePosition[] prevSwerveModulePositions;

    public SwerveOdom(SwerveKinematics kinematics) {
        this.kinematics = kinematics;

        currPos = new Pose2d(0,0, new Rotation2d(0));

        prevSwerveModulePositions = new SwerveModulePosition[]{null, null, null, null};

        for (int i=0; i < 4; ++i) {
            prevSwerveModulePositions[i] = new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
        }
    }

    public void setPose(Pose2d newPos) {
        currPos = newPos;
    }

    public void updatePoseWithGyro(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroAngle) {
        Twist2d twist = kinematics.getTwistFromDeltra(getSwerveModuleDelta(swerveModulePositions));
        twist.dtheta = gyroAngle.minus(currPos.getRotation()).getRadians();

        currPos = currPos.exp(twist);
    }

    private SwerveModulePosition[] getSwerveModuleDelta(SwerveModulePosition[] swerveModulePositions) {
        SwerveModulePosition[] states = new SwerveModulePosition[]{null, null, null, null};

        for (int i=0; i < 4; ++i) {
            states[i] = new SwerveModulePosition(
                (swerveModulePositions[i].distanceMeters - prevSwerveModulePositions[i].distanceMeters), 
                swerveModulePositions[i].angle
            );
        }

        prevSwerveModulePositions = swerveModulePositions;

        return states;
    }

    public Pose2d getPose() {
        return currPos;
    }
}
