package frc.robot.oi;

import frc.lib.Controller;

public class OI {
    private final Controller driverController;
    private final Controller operatorController;

    private static OI instance;

    public OI() {
        driverController = new Controller(0);
        operatorController = new Controller(1);
    }

    public Controller operatorController() {
        return operatorController;
    }

    public Controller driverController() {
        return driverController;
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }
}
