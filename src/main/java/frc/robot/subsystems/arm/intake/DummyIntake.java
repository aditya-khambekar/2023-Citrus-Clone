package frc.robot.subsystems.arm.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.subsystems.arm.constants.ArmConstants;

import java.util.function.Consumer;

public class DummyIntake extends SubsystemBase implements IIntake{
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
            case GROUND_INTAKING, SUBSTATION_INTAKING -> gamePiece == GamePiece.CONE?
                    IntakeState.INTAKING_CONE : IntakeState.INTAKING_CUBE;
            case IDLE -> IntakeState.IDLE;
            case LOW, MID, HIGH, OUTTAKING -> gamePiece == GamePiece.CONE?
                    IntakeState.OUTTAKING_CONE : IntakeState.OUTTAKING_CUBE;
        };
    }

    public static DummyIntake getInstance() {
        if (instance == null) {
            instance = new DummyIntake();
        }
        return instance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Intake Mode",
                intakeState::toString,
                (Consumer<String>) null);
    }
}
