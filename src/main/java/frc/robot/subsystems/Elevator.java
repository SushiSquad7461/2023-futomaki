package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.kElevator;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftElevator;
    private final CANSparkMax rightElevator;

    private final TunableNumber setpoint;

    private final ElevatorFeedforward ffd;
    private final ElevatorFeedforward ffu;
    private static boolean up;


    private static Elevator instance;
    private boolean resetElevator;
    
    private final LinearQuadraticRegulator<N2, N1, N1> lqr;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {
        double gearRatio = 9.0; // fuck ass random number
        double radius = 22.3 / 1000.0; // fuck ass random number in meters tho
        double mass = 17.5; // fuck ass random number in kg tho

        double velocityTerm = 
            (-1 * (gearRatio * gearRatio * DCMotor.getNEO(2).KtNMPerAmp)) / 
            (DCMotor.getNEO(2).rOhms * radius * radius * mass * DCMotor.getNEO(2).KvRadPerSecPerVolt);

        double voltageTerm = (gearRatio * DCMotor.getNEO(2).KtNMPerAmp) / (DCMotor.getNEO(2).rOhms * radius * mass);

        lqr = new LinearQuadraticRegulator<N2, N1, N1>(
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                0,1,0, velocityTerm
            ),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(
                0, voltageTerm
            ), 
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                1 / Math.pow(Units.inchesToMeters(0.8), 2),0,0, 1 / Math.pow(Units.inchesToMeters(4), 2)
            ), 
            Matrix.mat(Nat.N1(), Nat.N1()).fill(
                1 / Math.pow(12, 2)
            ),
            0.02 
        );

        // lqr = new LinearQuadraticRegulator<N2, N1, N1>(
        //     new LinearSystem<>(
        //         Matrix.mat(Nat.N2(), Nat.N2()).fill(
        //             0,1,0, velocityTerm
        //         ),
        //         Matrix.mat(Nat.N2(), Nat.N1()).fill(
        //         0, voltageTerm
        //         ),
        //         Matrix.mat(Nat.N1(), Nat.N2()).fill(1, 0),
        //         new Matrix<>(Nat.N1(), Nat.N1())
        //     ),
        //     VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(4.0)), 
        //     VecBuilder.fill(12.0), 
        //     0.02
        // );

        ffd = new ElevatorFeedforward(0, kElevator.kG_DOWN, 0);
        ffu = new ElevatorFeedforward(0, kElevator.kG_UP, 0);

        up = true;

        leftElevator = MotorHelper.createSparkMax(kElevator.LEFT_MOTOR_ID, MotorType.kBrushless, false, kElevator.CURRENT_LIMIT, IdleMode.kBrake);
        rightElevator = MotorHelper.createSparkMax(kElevator.RIGHT_MOTOR_ID, MotorType.kBrushless, true, kElevator.CURRENT_LIMIT, IdleMode.kBrake, Constants.kElevator.kP_UP, Constants.kElevator.kI, Constants.kElevator.kD, 0.0);

        leftElevator.follow(rightElevator, true);

        resetElevator = false;

        rightElevator.getEncoder().setPositionConversionFactor(1 / (31.856873 * 2.0));
        rightElevator.getEncoder().setVelocityConversionFactor((1 / (31.856873 * 2.0)) / 60.0);
      
        setpoint = new TunableNumber("Elavator Setpoint", kElevator.DEFUALT_VAL * 1 / (31.856873 * 2.0), Constants.kTuningMode);
    }

    public Command moveElevator(RobotState state) {
        return new SequentialCommandGroup(
            runOnce(
                () -> {
                    up = state.elevatorPos > getPose();
                    setpoint.setDefault(state.elevatorPos * (1 / (31.856873 * 2.0)));
                }
            ),
            new WaitUntilCommand(closeToSetpoint(state.elevatorPos * (1 / (31.856873 * 2.0))))
        );
    }

    public Command resetElevatorPoseStart() {
        return runOnce(
            () -> {
                rightElevator.set(-0.1);
                resetElevator = true;
            }
        );
    }

    public Command resetElevatorPoseEnd() {
        return runOnce(
            () -> {
                rightElevator.set(0.0);
                rightElevator.getEncoder().setPosition(0.0);
                resetElevator = false;
            }
        );
    }

    public double getError(double setpoint) {
        return Math.abs(rightElevator.getEncoder().getPosition() - setpoint);
    }

    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (getError(setpoint) < kElevator.MAX_ERROR);
    }

    public double getPose() {
        return rightElevator.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Setpoint", setpoint.get());
        SmartDashboard.putNumber("Elevator Position", rightElevator.getEncoder().getPosition());

        if (!resetElevator) {
            rightElevator.setVoltage(
                lqr.calculate(
                    Matrix.mat(Nat.N2(), Nat.N1()).fill(rightElevator.getEncoder().getPosition(), rightElevator.getEncoder().getVelocity()),
                    Matrix.mat(Nat.N2(), Nat.N1()).fill(setpoint.get(), 0)
                ).get(0, 0) + // SETPOINT IS NOW IN METERS REMBER THAT JOHN
                // ffd.calculate(0.0)
                (up ? ffu.calculate(0.0) : ffd.calculate(0.0))
            );
        }
    }
}
