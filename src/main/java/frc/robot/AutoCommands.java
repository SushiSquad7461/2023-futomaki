package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
    private SwerveAutoBuilder swerveAutoBuilder;

    private Manipulator manipulator;
    private Elevator elevator;
    private Swerve swerve;
    private SendableChooser<Command> chooser;
    private HashMap<String, Command> eventMap;



    public AutoCommands(Swerve swerve, Manipulator manipulator, Elevator elevator) {
        this.swerve = swerve;
        this.manipulator = manipulator;
        this.elevator = elevator;
        
        eventMap = new HashMap<String, Command>();
        eventMap.put("getCube", CommandFactory.setRobotState(manipulator, elevator, RobotState.GROUND_CUBE));
        eventMap.put("getCone", CommandFactory.setRobotState(manipulator, elevator, RobotState.GROUND_CONE));
        eventMap.put("scoreCone", CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE));
        eventMap.put("scoreCube", CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CUBE));
        eventMap.put("idle", CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE));
        eventMap.put("autoBalance", new AutoBalance());


        swerveAutoBuilder = new SwerveAutoBuilder(
            swerve::getOdomPose, 
            swerve::setOdomPose,
            kSwerve.TRANSLATION_CONSTANTS, 
            kSwerve.ROTATION_CONSTANTS, 
            swerve::drive, 
            eventMap,
            swerve
        );

        

        chooser = new SendableChooser<>();
        
        chooser.addOption("nothing", new InstantCommand(() -> {}));
        
        chooser.addOption("2pieceblue", new SequentialCommandGroup(
            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("2piece"),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE)
        ));

        chooser.addOption("2.5pieceblue", new SequentialCommandGroup(
            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("2piece"),

            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("2.5piece")
        ));

        chooser.addOption("2piecechargeblue", new SequentialCommandGroup(
            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("2piece"),

            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("2piececharge", kAuto.CHARGE_SPEED),
            new WaitCommand(kAuto.AUTO_BALANCE_WAIT),
            new AutoBalance()
        ));

        chooser.addOption("chargeblue", new SequentialCommandGroup(
            makeAuto("charge", kAuto.CHARGE_SPEED),
            new WaitCommand(kAuto.AUTO_BALANCE_WAIT),
            new AutoBalance()   
        ));

        chooser.addOption("2piecered", new SequentialCommandGroup(
            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("Red_2piece"),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE)
        ));

        chooser.addOption("2.5piecered", new SequentialCommandGroup(
            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("Red_2piece"),

            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("Red_2.5piece")
        ));

        chooser.addOption("2piecechargered", new SequentialCommandGroup(
            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("Red_2piece"),

            CommandFactory.setRobotState(manipulator, elevator, RobotState.L3_CONE),
            new WaitCommand(0.5),
            CommandFactory.setRobotState(manipulator, elevator, RobotState.IDLE),
            makeAuto("Red_2piececharge", kAuto.CHARGE_SPEED),
            new WaitCommand(kAuto.AUTO_BALANCE_WAIT),
            new AutoBalance()
        ));

        chooser.addOption("chargered", new SequentialCommandGroup(
            makeAuto("Red_charge", kAuto.CHARGE_SPEED),
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
