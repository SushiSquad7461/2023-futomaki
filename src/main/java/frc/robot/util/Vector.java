package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class Vector {
    public double x;
    public double y; 

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public double getScalar() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector rotate(Rotation2d angle) {
        double newAngle = Rotation2d.fromRadians(getAngle() -  MathUtil.inputModulus(angle.getRadians(), -Math.PI, Math.PI)).getRadians();
        double scalar = getScalar(); 

        this.x = Math.cos(newAngle) * scalar;
        this.y = Math.sin(newAngle) * scalar;

        return this;
    }
}
