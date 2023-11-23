package frc.robot;

import java.util.HashMap;

import javax.sound.midi.Sequencer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kAuto;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.manipulator.Manipulator;

public class AutoCommands {
    private SwerveAutoBuilder swerveAutoBuilder;
    private SendableChooser<Command> chooser;
    private HashMap<String, Command> eventMap;

    public AutoCommands(Swerve swerve, Manipulator manipulator, Elevator elevator) {
        eventMap = new HashMap<String, Command>();
        eventMap.put("getCube", CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.GROUND_CUBE));
        eventMap.put("getCone", CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.GROUND_CONE));
        eventMap.put("scoreCone", CommandFactory.setRobotStateElevatorFirst(manipulator, elevator, RobotState.L3_CONE));
        eventMap.put("scoreCube", CommandFactory.setRobotStateElevatorFirst(manipulator, elevator, RobotState.L3_CUBE));
        eventMap.put("idle", CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.IDLE));
        eventMap.put("autoBalance", new AutoBalance());

        swerveAutoBuilder = new SwerveAutoBuilder(
            swerve::getOdomPose, 
            swerve::setOdomPose,
            kSwerve.TRANSLATION_CONSTANTS, 
            kSwerve.ROTATION_CONSTANTS, 
            swerve::drive, 
            eventMap,
            true,
            swerve
        );

        chooser = new SendableChooser<>();
        
        chooser.addOption("nothing", new InstantCommand(() -> {}));

        chooser.addOption("Square", makeAuto("Square", 4.0));

        chooser.addOption("line", makeAuto("line", 4.0));


        chooser.addOption("One piece", new SequentialCommandGroup(
            new InstantCommand(() -> swerve.setOdomPose(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(180)))),
            CommandFactory.setRobotStateElevatorFirst(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            manipulator.reverseCurrentWrist(),
            new WaitCommand(0.5),
            CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.IDLE)
        ));   
        
        chooser.addOption("2 Piece", new SequentialCommandGroup(
            CommandFactory.setRobotStateElevatorFirst(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            manipulator.reverseCurrentWrist(),
            new WaitCommand(0.4),
            CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.IDLE),
            makeAuto("2piece"),
            CommandFactory.setRobotStateElevatorFirst(manipulator, elevator, RobotState.L3_CUBE),
            new WaitCommand(0.5),
            manipulator.reverseCurrentWrist(),
            new WaitCommand(0.5),
            CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.IDLE)
        ));

        chooser.addOption("3 Piece", new SequentialCommandGroup(
            CommandFactory.setRobotStateElevatorFirst(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.4),
            manipulator.reverseCurrentWrist(),
            new WaitCommand(0.3),
            CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.IDLE),
            makeAuto("2piece"),
            CommandFactory.setRobotStateElevatorFirst(manipulator, elevator, RobotState.L3_CUBE),
            new WaitCommand(0.25),
            manipulator.reverseCurrentWrist(),
            new WaitCommand(0.4),
            CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.IDLE),
            makeAuto("3 piece")
        ));

        chooser.addOption("Charge Blue", new SequentialCommandGroup(
            CommandFactory.setRobotStateElevatorFirst(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            manipulator.reverseCurrentWrist(),
            new WaitCommand(0.25),
            CommandFactory.setRobotStateWristFirst(manipulator, elevator, RobotState.IDLE),
            makeAuto("chargeWithoutLeavingCommunity", kAuto.CHARGE_SPEED),
            new WaitCommand(kAuto.AUTO_BALANCE_WAIT),
            new AutoBalance()
        ));                            

        SmartDashboard.putData("Auto Selecter", chooser);
    }

    private Command makeAuto(String path) {
        return swerveAutoBuilder.fullAuto(PathPlanner.loadPathGroup(path, kSwerve.MAX_SPEED, kSwerve.MAX_ACCELERATION));
    }

    private Command makeAuto(String path, double speed) {
        return swerveAutoBuilder.fullAuto(PathPlanner.loadPathGroup(path, speed, kSwerve.MAX_ACCELERATION));
    }

    public Command getAuto() {
        return chooser.getSelected();
    }
}
