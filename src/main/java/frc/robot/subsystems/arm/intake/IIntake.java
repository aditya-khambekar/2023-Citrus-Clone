package frc.robot.subsystems.arm.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.arm.constants.ArmConstants;

public interface IIntake extends Subsystem {
    Command setIntake(ArmConstants.ArmSuperstructureState state, GameConstants.GamePiece gamePiece);
}
