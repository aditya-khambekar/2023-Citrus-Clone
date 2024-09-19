package frc.robot.subsystems.arm.pivot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.util.Helpers;
import frc.robot.constants.GameConstants.GamePiece;

import java.util.function.DoubleConsumer;

public class Pivot extends SubsystemBase {
    private final CANSparkMax leftPivot, rightPivot;
    private final DutyCycleEncoder encoder;
    private final ProfiledPIDController pidController;
    private Trigger atWantedStateTrigger;

    private static Pivot instance;

    private Pivot() {
        leftPivot = new CANSparkMax(
                ArmConstants.IDs.LEFT_PIVOT_ID,
                CANSparkLowLevel.MotorType.kBrushless
        );
        rightPivot = new CANSparkMax(
                ArmConstants.IDs.RIGHT_PIVOT_ID,
                CANSparkLowLevel.MotorType.kBrushless
        );
        rightPivot.follow(leftPivot, true);

        encoder = new DutyCycleEncoder(
                ArmConstants.IDs.PIVOT_ENCODER_ID
        );
        pidController = new ProfiledPIDController(
                0.5, 0, 0,
                new TrapezoidProfile.Constraints(
                        0.5, 0.5
                )
        );
        atWantedStateTrigger = new Trigger(this::atState);
    }

    private void setPivot(ArmSuperstructureState state, GamePiece gamePiece) {
        pidController.setGoal(
                switch (state) {
                    case GROUND_INTAKING, IDLE -> ArmConstants.DOWN_ANGLE;
                    default -> gamePiece == GamePiece.CONE ?
                            ArmConstants.CONE_ANGLE :
                            ArmConstants.CUBE_ANGLE;
                });
    }

    private void runPivot() {
        leftPivot.set(pidController.calculate(
                encoder.getAbsolutePosition() * ArmConstants.PIVOT_SENSOR_TO_MECHANISM_RATIO
        ));
    }

    private boolean atState() {
        return Helpers.withinTolerance(
                pidController.getGoal().position,
                encoder.getAbsolutePosition() * ArmConstants.PIVOT_SENSOR_TO_MECHANISM_RATIO,
                ArmConstants.PIVOT_TOLERANCE);
    }

    public Command setPivotState(ArmSuperstructureState state, GamePiece gamePiece) {
        return runOnce(() -> setPivot(state, gamePiece))
                .andThen(this::runPivot);
    }

    public Trigger atWantedState() {
        return atWantedStateTrigger;
    }

    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Arm/Pivot/Position",
                () -> encoder.getAbsolutePosition() * ArmConstants.PIVOT_SENSOR_TO_MECHANISM_RATIO,
                (DoubleConsumer) null);
        builder.addDoubleProperty("Arm/Pivot/Position",
                () -> pidController.calculate(
                        encoder.getAbsolutePosition()
                                * ArmConstants.PIVOT_SENSOR_TO_MECHANISM_RATIO),
                (DoubleConsumer) null);
    }
}
