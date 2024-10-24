package frc.robot.subsystems.arm.elevator;

import frc.robot.constants.GameConstants;
import frc.robot.subsystems.arm.constants.ArmConstants;

public class SimElevatorSubsystem extends ElevatorSubsystem {
    private double position;

    public SimElevatorSubsystem() {
        position = ArmConstants.ElevatorConstants.DOWN_POSITION;
    }

    @Override
    public void setElevator(ArmConstants.ArmSuperstructureState state, GameConstants.GamePiece gamePiece) {
        position = switch (state) {
            case IDLE -> ArmConstants.ElevatorConstants.DOWN_POSITION;
            case GROUND_INTAKING -> ArmConstants.ElevatorConstants.GROUND_INTAKING_POSITION;
            case SUBSTATION_INTAKING -> ArmConstants.ElevatorConstants.SUBSTATION_INTAKING_POSITION;
            case LOW -> gamePiece == GameConstants.GamePiece.CONE ?
                    ArmConstants.ElevatorConstants.CONE_POSITIONS[0] :
                    ArmConstants.ElevatorConstants.CUBE_POSITIONS[0];
            case MID -> gamePiece == GameConstants.GamePiece.CONE ?
                    ArmConstants.ElevatorConstants.CONE_POSITIONS[1] :
                    ArmConstants.ElevatorConstants.CUBE_POSITIONS[1];
            case HIGH -> gamePiece == GameConstants.GamePiece.CONE ?
                    ArmConstants.ElevatorConstants.CONE_POSITIONS[2] :
                    ArmConstants.ElevatorConstants.CUBE_POSITIONS[2];
            default -> position;
        };
    }

    @Override
    public void runElevator() {

    }

    @Override
    protected boolean atState() {
        return true;
    }

    @Override
    public double getElevatorPosition() {
        return position;
    }
}
