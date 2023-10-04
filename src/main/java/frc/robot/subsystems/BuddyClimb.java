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
    private final TunableNumber setpoint;
    private final CANSparkMax motor;
    private final AbsoluteEncoder absoluteEncoder;
    private final PIDTuning Tuning;

    public BuddyClimb(){
        motor = MotorHelper.createSparkMax(Constants.kBuddyClimb.kMotorID, MotorType.kBrushless, false, 40, IdleMode.kBrake, Constants.kBuddyClimb.kP,Constants.kBuddyClimb.kI,Constants.kBuddyClimb.kD, Constants.kBuddyClimb.kF);
        setpoint = new TunableNumber("Setpoint", 0, Constants.kTuningMode);
        absoluteEncoder= new AbsoluteEncoder(0, Constants.kBuddyClimb.kOffset);

        Tuning = new PIDTuning(Constants.kBuddyClimb.kP,Constants.kBuddyClimb.kI,Constants.kBuddyClimb.kD,Constants.kTuningMode);
    
    }

    @Override
    public void periodic() {
        absoluteEncoder.getPosition();
        Tuning.updatePID(motor);

    }

    // public void defaultPos(){
    //     setArmSetPoint(90);
    // }
    // public void straight(){
    //     setArmSetPoint(180);
    // }
    // public void lift(){
    //     setArmSetPoint(170);

    // }
    public Command setArmSetPoint(double setPoint){
        return runOnce(() -> setpoint.setDefault(setPoint));
    }
}
