package frc.lib.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.robot.constants.Constants.DEADZONE_VALUE;

public class Controller extends XboxController implements Subsystem {
    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public Controller(int port) {
        super(port);
    }

    @Override
    public double getLeftX() {
        return applyDeadzone(super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return applyDeadzone(super.getLeftY());
    }

    @Override
    public double getRightX() {
        return applyDeadzone(super.getRightX());
    }

    @Override
    public double getRightY() {
        return applyDeadzone(super.getRightY());
    }

    private static double applyDeadzone(double value) {
        if (Math.abs(value) < DEADZONE_VALUE) {
            return 0;
        }
        return Math.signum(value) * ((Math.abs(value) - DEADZONE_VALUE) / (1 - DEADZONE_VALUE));
    }

    public boolean leftTrigger() {
        return super.getLeftTriggerAxis() >= 0.5;
    }

    public boolean rightTrigger() {
        return super.getRightTriggerAxis() >= 0.5;
    }

    public Command rumbleCommand() {
        return Commands.runOnce(
                () -> {
                    this.setRumble(RumbleType.kBothRumble, 0.3);
                }
        );
    }

    public Command stopRumbleCommand() {
        return Commands.runOnce(
                () -> {
                    this.setRumble(RumbleType.kBothRumble, 0);
                }
        );
    }

    public Command rumbleDurationCommand(double seconds) {
        return rumbleCommand().andThen(Commands.waitSeconds(seconds)).andThen(stopRumbleCommand());
    }
}
