package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import SushiFrcLib.SmartDashboard.AllianceColor;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

/**
 * Command that controls teleop swerve.
 */
public class TeleopSwerveDrive extends CommandBase {
    private final Swerve swerve;

    private final Supplier<Double> xaxisSupplier; 
    private final Supplier<Double> yaxisSupplier;
    private final Supplier<Double> rotSupplier;
    private final Supplier<Double> speedMultiplier;

    /**
     * Pass in defualt speed multiplier of 1.0
     */
    public TeleopSwerveDrive(Swerve swerve,
        Supplier<Double> xaxisSupplier, 
        Supplier<Double> yaxisSupplier, 
        Supplier<Double> rotSupplier
    ) {
        this(
            swerve,
            xaxisSupplier,
            yaxisSupplier,
            rotSupplier,
            () -> 1.0
        );
    }

    /**
     * Set swerve subsytem, controlers, axis's, and other swerve paramaters.
     */
    public TeleopSwerveDrive(Swerve swerve,
        Supplier<Double> xaxisSupplier, 
        Supplier<Double> yaxisSupplier, 
        Supplier<Double> rotSupplier,
        Supplier<Double> speedMultiplier
    ) {
        this.swerve = swerve;

        this.xaxisSupplier = xaxisSupplier;
        this.yaxisSupplier = yaxisSupplier;
        this.rotSupplier = rotSupplier;
        this.speedMultiplier = speedMultiplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forwardBack = yaxisSupplier.get() * speedMultiplier.get();
        double leftRight = -xaxisSupplier.get()  * speedMultiplier.get();
        double rot = (rotSupplier.get()) * speedMultiplier.get();

        forwardBack = Normalization.applyDeadband(forwardBack, Constants.STICK_DEADBAND);

        leftRight = Normalization.applyDeadband(leftRight, Constants.STICK_DEADBAND);

        Translation2d translation = new Translation2d(forwardBack, leftRight);

        rot = Normalization.cube(rot);
        rot *= kSwerve.MAX_ANGULAR_VELOCITY;

        swerve.driveWithRotationLock(
            (new Translation2d(Normalization.cube(translation.getNorm()), translation.getAngle())).times(kSwerve.MAX_SPEED), 
            rot
        );
    }
}
