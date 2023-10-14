package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kBuddyClimb;

public class BuddyClimb extends SubsystemBase {
    private final CANSparkMax anujIsASmallBitch;

    private static BuddyClimb instance;

    public static BuddyClimb getInstance() {
        if (instance == null) {
            instance = new BuddyClimb();
        }
        return instance;
    }

    private BuddyClimb() {
        anujIsASmallBitch = new CANSparkMax(kBuddyClimb.BUDDY_CLIMB_MOTOR_ID, MotorType.kBrushless);
        anujIsASmallBitch.setSmartCurrentLimit(40);
        anujIsASmallBitch.setIdleMode(IdleMode.kBrake);
        anujIsASmallBitch.setInverted(true);
    }

    public void setSpeed(double speed) {
        if (speed > 0) {
            anujIsASmallBitch.setSmartCurrentLimit(40);
        } else {
            anujIsASmallBitch.setSmartCurrentLimit(1);
        }
        anujIsASmallBitch.set(speed);
    }

    @Override
    public void periodic() {}
}
