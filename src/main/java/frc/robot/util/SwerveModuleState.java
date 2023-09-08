package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleState {
    public double velocity;  // meters per second
    public Rotation2d angle;

    public SwerveModuleState(double velocity, Rotation2d angle) {
        this.velocity = velocity;
        this.angle = angle;
    }
}
