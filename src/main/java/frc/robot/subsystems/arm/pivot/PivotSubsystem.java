package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.GameConstants.*;
import frc.robot.subsystems.arm.constants.ArmConstants.*;

import java.util.Objects;

public abstract class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;

    public static PivotSubsystem getInstance() {
        if (instance == null) {
            instance = Robot.isReal() ? new ConcretePivotSubsystem() : new SimPivotSubsystem();
        }
        return instance;
    }

    public abstract void setPivot(ArmSuperstructureState state, GamePiece gamePiece);

    public abstract void runPivot();

    protected abstract boolean atState();

    public Trigger atRequestedStateTrigger() {
        return new Trigger(this::atState);
    }

    public abstract Rotation2d getAngle();
}
