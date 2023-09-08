package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModulePosition {
    public final double distance;  // meters
    public final Rotation2d angle;

    public SwerveModulePosition(double distance, Rotation2d angle) {
        this.distance = distance;
        this.angle = angle;
    }
}

