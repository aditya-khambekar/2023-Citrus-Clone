package frc.robot.subsystems.arm.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GameConstants;

public interface IWrist extends Subsystem {
    Trigger atWantedState();

    Command setWristState(GameConstants.GamePiece gamePiece);
}
