package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.elevator.Elevator;
import frc.robot.subsystems.arm.intake.DummyIntake;
import frc.robot.subsystems.arm.intake.IIntake;
import frc.robot.subsystems.arm.pivot.Pivot;
import frc.robot.subsystems.arm.wrist.DummyWrist;
import frc.robot.subsystems.arm.wrist.IWrist;

public class ArmSuperstructure extends SubsystemBase {
    private final Pivot pivot;
    private final Elevator elevator;
    private final IWrist wrist;
    private final IIntake intake;

    private static ArmSuperstructure instance;

    public ArmSuperstructure() {
        pivot = Pivot.getInstance();
        elevator = Elevator.getInstance();
        wrist = DummyWrist.getInstance();
        intake = DummyIntake.getInstance();
    }

    public Command setStateCommand(ArmSuperstructureState state, GamePiece gamePiece) {
        return Commands.parallel(
                pivot.setPivotState(state, gamePiece).until(pivot.atWantedState()),
                elevator.setElevatorState(state, gamePiece).until(elevator.atWantedState()),
                wrist.setWristState(state, gamePiece)
        ).andThen(
                intake.setIntake(state, gamePiece).deadlineWith(new WaitCommand(2))
        );
    }

    public Trigger atWantedState() {
        return pivot.atWantedState().and(elevator.atWantedState()).and(wrist.atWantedState());
    }

    public static ArmSuperstructure getInstance() {
        if (instance == null) {
            instance = new ArmSuperstructure();
        }
        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(pivot);
        SmartDashboard.putData(elevator);
        SmartDashboard.putData(wrist);
        SmartDashboard.putData(intake);
    }
}
