package frc.robot.subsystems.arm.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.util.Helpers;
import frc.robot.constants.GameConstants.GamePiece;

public class Elevator extends SubsystemBase {
    private final TalonFX leftElevator, rightElevator;
    private final CANcoder encoder;
    private MotionMagicVoltage controlRequest;
    private Trigger atWantedStateTrigger;

    private static Elevator instance;

    private Elevator() {
        leftElevator = new TalonFX(ArmConstants.IDs.LEFT_ELEVATOR_ID);
        rightElevator = new TalonFX(ArmConstants.IDs.RIGHT_ELEVATOR_ID);
        encoder = new CANcoder(ArmConstants.IDs.ELEVATOR_ENCODER_ID);
        var leftConfigurator = leftElevator.getConfigurator();
        TalonFXConfiguration leftConfiguration = new TalonFXConfiguration()
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(0.5)
                                .withMotionMagicCruiseVelocity(0.8))
                .withFeedback(
                        new FeedbackConfigs()
                                .withRemoteCANcoder(encoder)
                                .withRotorToSensorRatio(ArmConstants.ELEVATOR_ROTOR_TO_SENSOR_RATIO)
                                .withSensorToMechanismRatio(ArmConstants.ELEVATOR_SENSOR_TO_MECHANISM_RATIO))
                .withSlot0(
                        new Slot0Configs()
                                .withGravityType(GravityTypeValue.Elevator_Static)
                                .withKP(0.5)
                                .withKI(0)
                                .withKD(0)
                                .withKG(0.1)
                );
        leftConfigurator.apply(leftConfiguration);
        rightElevator.setControl(new Follower(ArmConstants.IDs.LEFT_ELEVATOR_ID, true));
        controlRequest = new MotionMagicVoltage(ArmConstants.DOWN_POSITION);
        atWantedStateTrigger = new Trigger(this::atState);
    }

    private void setElevator(ArmSuperstructureState state, GamePiece gamePiece) {
        controlRequest.Position = switch (state) {
            case IDLE -> ArmConstants.DOWN_POSITION;
            case INTAKING -> ArmConstants.INTAKING_POSITION;
            case LOW -> gamePiece == GamePiece.CONE ?
                    ArmConstants.CONE_POSITIONS[0] :
                    ArmConstants.CUBE_POSITIONS[0];
            case MID -> gamePiece == GamePiece.CONE ?
                    ArmConstants.CONE_POSITIONS[1] :
                    ArmConstants.CUBE_POSITIONS[1];
            case HIGH -> gamePiece == GamePiece.CONE ?
                    ArmConstants.CONE_POSITIONS[2] :
                    ArmConstants.CUBE_POSITIONS[2];
        };
    }

    private void runElevator() {
        leftElevator.setControl(controlRequest);
    }

    private boolean atState() {
        return Helpers.withinTolerance(
                controlRequest.Position,
                leftElevator.getPosition().getValue(),
                ArmConstants.ELEVATOR_TOLERANCE
        );
    }

    public Command setElevatorState(ArmSuperstructureState state, GamePiece gamePiece) {
        return runOnce(() -> setElevator(state, gamePiece))
                .andThen(run(this::runElevator));
    }

    public Trigger atWantedState() {
        return atWantedStateTrigger;
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
}
