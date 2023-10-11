package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    private Boolean isRedAlliance;
    private NetworkTable table;

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
        
        table = NetworkTableInstance.getDefault().getTable("FMSInfo");
        isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);

        double forwardBack = yaxisSupplier.get() * (isRedAlliance ? -1 : 1);
        double leftRight = -xaxisSupplier.get() * (isRedAlliance ? -1 : 1);
        double rot = rotSupplier.get();

        forwardBack = Normalization.cube(applyDeadband(forwardBack));

        leftRight = Normalization.cube(applyDeadband(leftRight));

        Translation2d translation = new Translation2d(forwardBack, leftRight)
                .times(kSwerve.MAX_SPEED);

        rot = Normalization.cube(rot);
        rot *= kSwerve.MAX_ANGULAR_VELOCITY;

        swerve.drive(new Translation2d(translation.getX(), translation.getY()), rot);
    }

    private double applyDeadband(double initalVal) {
        return Math.abs(initalVal) <  Constants.STICK_DEADBAND ? 0 : (
            (initalVal - ((initalVal < 0 ? -1 : 1) * Constants.STICK_DEADBAND)) 
            / (1 - Constants.STICK_DEADBAND));
    }
}
