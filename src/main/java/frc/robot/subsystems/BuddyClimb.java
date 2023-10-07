package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Constants.kBuddyClimb;

public class BuddyClimb extends SubsystemBase {
    private final CANSparkMax anujIsASmallBitch;

    public BuddyClimb() {
        anujIsASmallBitch = new CANSparkMax(kBuddyClimb.BUDDY_CLIMB_MOTOR_ID, MotorType.kBrushless);
        anujIsASmallBitch.setSmartCurrentLimit(40);
        anujIsASmallBitch.setIdleMode(IdleMode.kBrake);
    }

    public void setSpeed(double speed) {
        if (speed > 0) {
            anujIsASmallBitch.setSmartCurrentLimit(5);
        } else {
            anujIsASmallBitch.setSmartCurrentLimit(50);
        }
        anujIsASmallBitch.set(speed);
    }

    @Override
    public void periodic() {
        setSpeed(OI.getInstance().getOperatorController().getLeftY() * 0.4);
    }

}
