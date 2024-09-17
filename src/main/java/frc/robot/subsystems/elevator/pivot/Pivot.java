package frc.robot.subsystems.elevator.pivot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.constants.ElevatorConstants.ElevatorSuperstructureState;

public class Pivot extends SubsystemBase {
    private final TalonFX leftPivot, rightPivot;
    private final CANcoder encoder;

    private static Pivot instance;

    private Pivot() {
        leftPivot = new TalonFX(ElevatorConstants.LEFT_PIVOT_ID);
        rightPivot = new TalonFX(ElevatorConstants.RIGHT_PIVOT_ID);
        encoder = new CANcoder(ElevatorConstants.ENCODER_ID);
        var leftConfigurator = leftPivot.getConfigurator();
        TalonFXConfiguration leftConfig = new TalonFXConfiguration()
                .withFeedback(
                        new FeedbackConfigs()
                                .withRemoteCANcoder(encoder)
                                .withRotorToSensorRatio(ElevatorConstants.ROTOR_TO_SENSOR_RATIO)
                                .withSensorToMechanismRatio(ElevatorConstants.SENSOR_TO_MECHANISM_RATIO))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(0.5)
                                .withMotionMagicCruiseVelocity(0.8)
                );
        leftConfigurator.apply(leftConfig);
        rightPivot.setControl(new Follower(ElevatorConstants.LEFT_PIVOT_ID, true));
    }

    public void setPivot(ElevatorSuperstructureState state) {
        leftPivot.setControl(new MotionMagicVoltage(
                switch (state) {
                    case INTAKING, IDLE -> ElevatorConstants.DOWN_ANGLE;
                    case CONE_LOW, CONE_MID, CONE_HIGH -> ElevatorConstants.CONE_ANGLE;
                    case CUBE_LOW, CUBE_MID, CUBE_HIGH -> ElevatorConstants.CUBE_ANGLE;
                }
        ));
    }

    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }
}
