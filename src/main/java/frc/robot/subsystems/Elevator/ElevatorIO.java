package frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

import SushiFrcLib.SmartDashboard.PIDTuning;

public abstract class ElevatorIO {
    protected final ElevatorIOInputs inputs;

    @AutoLog
    public static class ElevatorIOInputs {
        public double previouseSetpoint;
        public double elevatorSetpoint;

        public boolean up;

        public double kG;
        public double kP;

        public boolean speedMode;
        public double speed;
    }

    public ElevatorIO() {
        this.inputs = new ElevatorIOInputs();
    }

    public abstract void setSetpoint(double setpoint);

    public abstract double getPosition();

    public abstract void updatePID(PIDTuning pid);

    public abstract void setPosition(double position);

    public void logInputs() {
        Logger.processInputs("Elevator", inputs);
    }

    public abstract void applySpeed(double speed);

    public abstract BooleanSupplier closeToSetpoint(double setpoint);
}
