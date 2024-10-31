package frc.robot.subsystems.arm.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.subsystems.arm.constants.ArmPIDs;
import frc.robot.util.Helpers;
import frc.robot.constants.GameConstants.GamePiece;

public class ConcreteElevatorSubsystem extends ElevatorSubsystem {
    private final TalonFX leftElevator, rightElevator;
    private final MotionMagicDutyCycle controlRequest;

    public ConcreteElevatorSubsystem() {
        leftElevator = new TalonFX(ArmConstants.IDs.LEFT_ELEVATOR_ID);
        rightElevator = new TalonFX(ArmConstants.IDs.RIGHT_ELEVATOR_ID);
        var leftConfigurator = leftElevator.getConfigurator();
        TalonFXConfiguration leftConfiguration = new TalonFXConfiguration()
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(ArmPIDs.elevatorAcceleration.get())
                                .withMotionMagicCruiseVelocity(ArmPIDs.elevatorVelocity.get()))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ArmPIDs.elevatorKp.get())
                                .withKI(ArmPIDs.elevatorKi.get())
                                .withKD(ArmPIDs.elevatorKd.get())
                );
        leftConfigurator.apply(leftConfiguration);
        rightElevator.setControl(new Follower(ArmConstants.IDs.LEFT_ELEVATOR_ID, true));
        controlRequest = new MotionMagicDutyCycle(leftElevator.getPosition().getValueAsDouble());
    }

    public void setElevator(ArmSuperstructureState state, GamePiece gamePiece) {
        controlRequest.Position = getElevatorPosition(state, gamePiece);
    }

    public void runElevator() {
//        only uncomment this when the upper and lower positions are initialized
//        leftElevator.setControl(controlRequest);
    }

    protected boolean atState() {
        return Helpers.withinTolerance(
                controlRequest.Position,
                leftElevator.getPosition().getValueAsDouble(),
                ArmConstants.ElevatorConstants.ELEVATOR_TOLERANCE
        );
    }

    public double getCurrentPosition() {
        return leftElevator.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        leftElevator.getConfigurator().apply(
                new Slot0Configs()
                        .withKP(ArmPIDs.elevatorKp.get())
                        .withKI(ArmPIDs.elevatorKi.get())
                        .withKD(ArmPIDs.elevatorKd.get())
        );
        rightElevator.getConfigurator().apply(
                new Slot0Configs()
                        .withKP(ArmPIDs.elevatorKp.get())
                        .withKI(ArmPIDs.elevatorKi.get())
                        .withKD(ArmPIDs.elevatorKd.get())
        );
    }

    @Override
    public double getTargetPosition() {
        return controlRequest.Position;
    }
}
