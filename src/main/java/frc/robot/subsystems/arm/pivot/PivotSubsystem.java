package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.GameConstants;
import frc.robot.constants.GameConstants.*;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmConstants.*;

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

    public abstract double getCurrentRotation();

    public static double getTargetPosition(ArmSuperstructureState state, GamePiece gamePiece) {
        return switch (state) {
            case IDLE -> ArmConstants.PivotConstants.DOWN_ANGLE;
            case GROUND_INTAKING -> ArmConstants.PivotConstants.GROUND_INTAKING_ANGLE;
            case SUBSTATION_INTAKING -> ArmConstants.PivotConstants.SUBSTATION_INTAKING_ANGLE;
            default -> gamePiece == GameConstants.GamePiece.CONE ?
                    ArmConstants.PivotConstants.CONE_PIVOT_ANGLE :
                    ArmConstants.PivotConstants.CUBE_PIVOT_ANGLE;
        };
    }
}
