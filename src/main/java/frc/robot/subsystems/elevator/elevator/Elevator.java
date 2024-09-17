package frc.robot.subsystems.elevator.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.constants.ElevatorConstants.ElevatorSuperstructureState;

public class Elevator extends SubsystemBase {
    private final TalonFX leftElevator, rightElevator;

    private static Elevator instance;

    private Elevator() {
        leftElevator = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_ID);
        rightElevator = new TalonFX(ElevatorConstants.RIGHT_PIVOT_ID);
        var leftConfigurator = leftElevator.getConfigurator();
        TalonFXConfiguration leftConfiguration = new TalonFXConfiguration()
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(0.5)
                                .withMotionMagicCruiseVelocity(0.8)
                );
        leftConfigurator.apply(leftConfiguration);
        rightElevator.setControl(new Follower(ElevatorConstants.LEFT_ELEVATOR_ID, true));
    }

    private void setElevator(ElevatorSuperstructureState state) {
        leftElevator.setControl(new MotionMagicVoltage(
                switch (state) {
                    case IDLE -> ElevatorConstants.DOWN_POSITION;
                    case INTAKING -> ElevatorConstants.INTAKING_POSITION;
                    case CONE_LOW -> ElevatorConstants.CONE_POSITIONS[0];
                    case CONE_MID -> ElevatorConstants.CONE_POSITIONS[1];
                    case CONE_HIGH -> ElevatorConstants.CONE_POSITIONS[2];
                    case CUBE_LOW -> ElevatorConstants.CUBE_POSITIONS[0];
                    case CUBE_MID -> ElevatorConstants.CUBE_POSITIONS[1];
                    case CUBE_HIGH -> ElevatorConstants.CUBE_POSITIONS[2];
                }
        ));
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
}
