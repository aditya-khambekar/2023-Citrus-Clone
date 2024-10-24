package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.arm.constants.ArmConstants;

public class SimPivotSubsystem extends PivotSubsystem {
    private double position;

    public SimPivotSubsystem() {
        position = ArmConstants.PivotConstants.DOWN_ANGLE;
    }

    @Override
    public void setPivot(ArmConstants.ArmSuperstructureState state, GameConstants.GamePiece gamePiece) {
        position = switch (state) {
            case IDLE -> ArmConstants.PivotConstants.DOWN_ANGLE;
            case GROUND_INTAKING -> ArmConstants.PivotConstants.GROUND_INTAKING_ANGLE;
            case SUBSTATION_INTAKING -> ArmConstants.PivotConstants.SUBSTATION_INTAKING_ANGLE;
            default -> gamePiece == GameConstants.GamePiece.CONE ?
                    ArmConstants.PivotConstants.CONE_PIVOT_ANGLE :
                    ArmConstants.PivotConstants.CUBE_PIVOT_ANGLE;
        };
    }

    @Override
    public void runPivot() {

    }

    @Override
    protected boolean atState() {
        return true;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(position);
    }
}
