package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BuddyClimb extends SubsystemBase {
    private final TunableNumber speed;
    private final CANSparkMax motor;

    public BuddyClimb(){
        motor = MotorHelper.createSparkMax(Constants.kBuddyClimb.MOTOR_ID, MotorType.kBrushless, Constants.kBuddyClimb.INVERTED, Constants.kBuddyClimb.CURRENT_LIMIT, IdleMode.kBrake);
        speed = new TunableNumber("Speed", 0, Constants.kTuningMode);
    }

    @Override
    public void periodic() {
    }
    public Command setArmSpeed(double speed){
        return runOnce(() -> motor.set(speed));
    }
}
