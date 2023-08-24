package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI {
    private static OI instance;
    private CommandXboxController driverController;


    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    private OI() {
        driverController = new CommandXboxController(0);
    }

    public CommandXboxController getDriverController() {
        return driverController;
    }
}
