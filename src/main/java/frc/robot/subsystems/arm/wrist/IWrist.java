package frc.robot.subsystems.arm.wrist;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.arm.constants.ArmConstants;

public interface IWrist extends Subsystem, Sendable {
    Trigger atWantedState();

    Command setWristState(ArmConstants.ArmSuperstructureState state, GameConstants.GamePiece gamePiece);
}
