package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kBuddyClimb;

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
        buddyClimb = new CANSparkMax(kBuddyClimb.BUDDY_CLIMB_MOTOR_ID, MotorType.kBrushless);
        buddyClimb.setSmartCurrentLimit(kBuddyClimb.UP_CURRENT_LIMIT);
        buddyClimb.setIdleMode(IdleMode.kBrake);
        buddyClimb.setInverted(true);
    }

    public void setSpeed(double speed) {
        buddyClimb.setSmartCurrentLimit(speed > 0 ? kBuddyClimb.UP_CURRENT_LIMIT : kBuddyClimb.DOWN_CURRENT_LIMIT);
        buddyClimb.set(speed);
    }
}
