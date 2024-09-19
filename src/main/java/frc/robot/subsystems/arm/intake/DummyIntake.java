package frc.robot.subsystems.arm.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.subsystems.arm.constants.ArmConstants;

public class DummyIntake extends SubsystemBase implements IIntake{
    private ArmConstants.ArmSuperstructureState superstructureState;

    private enum IntakeState {
        INTAKING_CUBE,
        INTAKING_CONE,
        IDLE,
        OUTTAKING_CUBE,
        OUTTAKING_CONE,
    }

    private IntakeState intakeState;

    private static DummyIntake instance;

    public DummyIntake() {
        intakeState = IntakeState.IDLE;
    }

    @Override
    public Command setIntake(ArmConstants.ArmSuperstructureState state, GamePiece gamePiece) {
        return run(() -> setState(state, gamePiece));
    }

    private void setState(ArmSuperstructureState state, GamePiece gamePiece) {
        intakeState = switch (state) {
            case INTAKING -> gamePiece == GamePiece.CONE?
                    IntakeState.INTAKING_CONE : IntakeState.INTAKING_CUBE;
            case IDLE -> IntakeState.IDLE;
            case LOW, MID, HIGH -> gamePiece == GamePiece.CONE?
                    IntakeState.OUTTAKING_CONE : IntakeState.OUTTAKING_CUBE;
        };
    }

    public static DummyIntake getInstance() {
        if (instance == null) {
            instance = new DummyIntake();
        }
        return instance;
    }
}
