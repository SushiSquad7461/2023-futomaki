package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

/**
 * Command that controls teleop swerve.
 */
// TODO could prolly add this to sushi lib
public class TeleopSwerveDrive extends CommandBase {
    private final Swerve swerve;

    private final Supplier<Double> xAxisSupplier; 
    private final Supplier<Double> yAxisSupplier;
    private final Supplier<Double> rotSupplier;
    private final Supplier<Double> speedMultiplier;

    /**
     * Pass in defualt speed multiplier of 1.0
     */
    public TeleopSwerveDrive(Swerve swerve,
        Supplier<Double> xAxisSupplier, 
        Supplier<Double> yAxisSupplier, 
        Supplier<Double> rotSupplier
    ) {
        this(
            swerve,
            xAxisSupplier,
            yAxisSupplier,
            rotSupplier,
            () -> 1.0
        );
    }

    /**
     * Set swerve subsytem, controlers, axis's, and other swerve paramaters.
     */
    public TeleopSwerveDrive(Swerve swerve,
        Supplier<Double> xAxisSupplier, 
        Supplier<Double> yAxisSupplier, 
        Supplier<Double> rotSupplier,
        // TODO why not just have the max speed be passed in directly
        Supplier<Double> speedMultiplier
    ) {
        this.swerve = swerve;

        this.xAxisSupplier = xAxisSupplier;
        this.yAxisSupplier = yAxisSupplier;
        this.rotSupplier = rotSupplier;
        this.speedMultiplier = speedMultiplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forwardBack = yAxisSupplier.get() * speedMultiplier.get();
        double leftRight = xAxisSupplier.get()  * speedMultiplier.get();
        double rot = rotSupplier.get() * speedMultiplier.get();

        forwardBack = Normalization.applyDeadband(forwardBack, Constants.STICK_DEADBAND);
        leftRight = Normalization.applyDeadband(leftRight, Constants.STICK_DEADBAND);

        Translation2d translation = new Translation2d(forwardBack, leftRight);

        // TODO more of a sushi lib thing but this is not a normalization, if anything just change this to calling the math pow directly
        rot = Normalization.cube(rot);
        rot *= kSwerve.MAX_ANGULAR_VELOCITY;

        swerve.driveWithRotationLock(
            (new Translation2d(Normalization.cube(translation.getNorm()), translation.getAngle())).times(kSwerve.MAX_SPEED), 
            rot
        );
    }
}
