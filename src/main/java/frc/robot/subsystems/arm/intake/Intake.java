package frc.robot.subsystems.arm.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;

public class Intake extends SubsystemBase implements IIntake {
    private final TalonFX intake;

    private Intake instance;

    public Intake() {
        intake = new TalonFX(ArmConstants.IDs.INTAKE_ID);
    }

    private void runIntake(ArmSuperstructureState state, GamePiece gamePiece) {
        intake.setControl(new DutyCycleOut(switch (state) {
            case IDLE -> 0;
            case INTAKING -> gamePiece == GamePiece.CONE?
                    ArmConstants.INTAKE_SPEED:
                    -ArmConstants.INTAKING_POSITION;
            default -> gamePiece == GamePiece.CONE?
                    -ArmConstants.INTAKE_SPEED:
                    ArmConstants.INTAKE_SPEED;
        }));
    }

    public Command setIntake(ArmSuperstructureState state, GamePiece gamePiece) {
        return run(() -> runIntake(state, gamePiece));
    }
}
