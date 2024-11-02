package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.elevator.ElevatorSubsystem;
import frc.robot.subsystems.arm.pivot.PivotSubsystem;

public class ArmSuperstructure extends SubsystemBase {
    private final PivotSubsystem pivot;
    private final ElevatorSubsystem elevator;

    private static ArmSuperstructure instance;

    private final Mechanism2d armMech;
    private final MechanismRoot2d armRoot;
    private final MechanismLigament2d arm;

    public ArmSuperstructure() {
        pivot = PivotSubsystem.getInstance();
        elevator = ElevatorSubsystem.getInstance();

        armMech = new Mechanism2d(20, 20);
        armRoot = armMech.getRoot("Arm Root", 2, 2);
        arm = armRoot.append(
                new MechanismLigament2d(
                        "Arm",
                        getElevatorLength(elevator.getCurrentPosition()),
                        getPivotDegrees(pivot.getCurrentRotation())
                )
        );
    }

    public Command setStateCommand(ArmSuperstructureState state, GamePiece gamePiece) {
        return runOnce(() -> {
            elevator.setElevator(state, gamePiece);
            pivot.setPivot(state, gamePiece);
        }).andThen(
                Commands.run(() -> {
                    pivot.runPivot();
                    elevator.runElevator();
                })
        ).until(atWantedState());
    }

    public Trigger atWantedState() {
        return pivot.atRequestedStateTrigger().and(elevator.atRequestedStateTrigger());
    }

    public static ArmSuperstructure getInstance() {
        if (instance == null) {
            instance = new ArmSuperstructure();
        }
        return instance;
    }

    public static double getPivotDegrees(double rotation) {
        return (ArmConstants.PivotConstants.DOWN_ANGLE - rotation) * 360;
    }

    public static double getElevatorLength(double position) {
        return position * 10;
    }

    public static double getPivotRadians(double rotation) {
        return Units.degreesToRadians(getPivotDegrees(rotation));
    }

    @Override
    public void periodic() {
        arm.setAngle(Units.radiansToDegrees(pivot.getCurrentRotation()));
        arm.setLength(getElevatorLength(elevator.getCurrentPosition()));
        SmartDashboard.putData("arm mech", armMech);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm Superstructure");
        builder.addDoubleProperty("Elevator Position",
                elevator::getCurrentPosition,
                null
        );
        builder.addDoubleProperty("Elevator Target Position",
                elevator::getTargetPosition,
                null
        );
        builder.addDoubleProperty("Pivot Angle",
                pivot::getCurrentRotation,
                null
        );
    }
}
