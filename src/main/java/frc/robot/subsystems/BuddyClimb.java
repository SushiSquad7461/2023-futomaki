package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BuddyClimb extends SubsystemBase {
    private final CANSparkMax buddyClimb;

    private static BuddyClimb instance;

    public static BuddyClimb getInstance() {
        if (instance == null) {
            instance = new BuddyClimb();
        }
        return instance;
    }

    private BuddyClimb() {
        buddyClimb = Constants.BuddyClimb.MOTOR_CONFIG.createSparkMax();
    }

    public void setSpeed(double speed) {
        buddyClimb.setSmartCurrentLimit(speed > 0 ? Constants.BuddyClimb.UP_CURRENT_LIMIT : Constants.BuddyClimb.UP_CURRENT_LIMIT);
        buddyClimb.set(speed);
    }
}
