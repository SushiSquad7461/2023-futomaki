package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class BuddyClimb extends SubsystemBase {
    private final CANSparkMax anujIsASmallBitch;

    public BuddyClimb() {
        anujIsASmallBitch = new CANSparkMax(30, MotorType.kBrushless);
        anujIsASmallBitch.setSmartCurrentLimit(40);
        anujIsASmallBitch.setIdleMode(IdleMode.kBrake);
    } 

    public void setSpeed(double speed) {
        anujIsASmallBitch.set(speed);
    }

    @Override
    public void periodic() {
        setSpeed(OI.getInstance().getOperatorController().getLeftY());
    }

}
