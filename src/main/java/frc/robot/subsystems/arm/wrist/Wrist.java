package frc.robot.subsystems.arm.wrist;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.util.Helpers;

public class Wrist extends SubsystemBase implements IWrist {
    private final TalonFX wrist;
    private final MotionMagicVoltage controlRequest;
    private Trigger atWantedStateTrigger;

    private static Wrist instance;

    private Wrist() {
        wrist = new TalonFX(ArmConstants.IDs.WRIST_ID);
        var wristConfigurator = wrist.getConfigurator();
        TalonFXConfiguration wristConfig = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withGravityType(
                                        GravityTypeValue.Arm_Cosine
                                )
                                .withKP(0.5)
                                .withKI(0)
                                .withKD(0)
                                .withKG(0.1))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(0.5)
                                .withMotionMagicAcceleration(0.5))
                .withFeedback(
                        new FeedbackConfigs()
                                .withFeedbackSensorSource(
                                        FeedbackSensorSourceValue.FusedCANcoder
                                )
                );
        wristConfigurator.apply(wristConfig);
        controlRequest = new MotionMagicVoltage(ArmConstants.IDLE_WRIST_ANGLE);
        atWantedStateTrigger = new Trigger(this::atState);
    }

    private void setWrist(GamePiece gamePiece) {
        controlRequest.Position = switch (gamePiece) {
            case CONE -> ArmConstants.CONE_ANGLE;
            case CUBE -> ArmConstants.CUBE_ANGLE;
        };
    }

    private void runWrist() {
        wrist.setControl(controlRequest);
    }

    private boolean atState() {
        return Helpers.withinTolerance(
                controlRequest.Position,
                wrist.getPosition().getValue(),
                ArmConstants.WRIST_TOLERANCE
        );
    }

    public Trigger atWantedState() {
        return atWantedStateTrigger;
    }

    public Command setWristState(GamePiece gamePiece) {
        return runOnce(() -> setWrist(gamePiece))
                .andThen(run(this::runWrist));
    }
}
