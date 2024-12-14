package frc.robot.subsystems.arm;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.subsystems.arm.elevator.ElevatorSubsystem;
import frc.robot.subsystems.arm.pivot.PivotSubsystem;

public class ArmSuperstructure extends SubsystemBase {
    private final PivotSubsystem pivot;
    private final ElevatorSubsystem elevator;

    private static ArmSuperstructure instance;

    private ArmSuperstructureState currentState, prevState;

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
        currentState = prevState = ArmSuperstructureState.IDLE;
        elevator.setElevator(ArmSuperstructureState.IDLE, GamePiece.CUBE);
        pivot.setPivot(ArmSuperstructureState.IDLE, GamePiece.CUBE);
    }

    public Command setStateCommand(ArmSuperstructureState state, GamePiece gamePiece) {
        return Commands.runOnce(
                () -> {
                    prevState = currentState;
                    currentState = state;
                },
                this
        ).andThen(
                Commands.either(
                        Commands.sequence(
                                Commands.runOnce(() -> elevator.setElevator(state, gamePiece)),
                                Commands.waitUntil(elevator.atRequestedStateTrigger()),
                                Commands.runOnce(() -> pivot.setPivot(state, gamePiece)),
                                Commands.waitUntil(pivot.atRequestedStateTrigger())
                        ),
                        Commands.sequence(
                                Commands.runOnce(() -> pivot.setPivot(state, gamePiece)),
                                Commands.waitUntil(pivot.atRequestedStateTrigger()),
                                Commands.runOnce(() -> elevator.setElevator(state, gamePiece)),
                                Commands.waitUntil(elevator.atRequestedStateTrigger())
                        ),
                        () -> prevState.high
                )
        );
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
        return (ArmConstants.PivotConstants.DOWN_ANGLE - rotation) * 360 - 180;
    }

    public static double getPivotRadians(double rotation) {
        return Units.degreesToRadians(getPivotDegrees(rotation));
    }

    public static double getElevatorLength(double position) {
        return (ArmConstants.ElevatorConstants.DOWN_POSITION - position) * 3 / 4 - 6;
    }

    public static double getElevatorSetpoint(double length) {
        return (length + 6) * 4 / 3 - ArmConstants.ElevatorConstants.DOWN_POSITION;
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
        builder.addDoubleProperty(
                "Elevator Position",
                elevator::getCurrentPosition,
                null
        );
        builder.addDoubleProperty(
                "Elevator Target Position",
                elevator::getTargetPosition,
                null
        );
        builder.addDoubleProperty(
                "Pivot Angle",
                pivot::getCurrentRotation,
                null
        );
        builder.addDoubleProperty(
                "Pivot Target Angle",
                pivot::getTargetRotation,
                null
        );
        builder.addStringProperty(
                "State",
                () -> currentState.name(),
                null
        );
        builder.addBooleanProperty(
                "Elevator Within Tolerance",
                elevator.atRequestedStateTrigger(),
                null
        );
        builder.addBooleanProperty(
                "Pivot Within Tolerance",
                pivot.atRequestedStateTrigger(),
                null
        );
    }

    public Command elevatorQuasistatic(SysIdRoutine.Direction direction) {
        return elevator.quasistatic(direction);
    }

    public Command elevatorDynamic(SysIdRoutine.Direction direction) {
        return elevator.dynamic(direction);
    }
}
