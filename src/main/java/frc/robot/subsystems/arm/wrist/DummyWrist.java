package frc.robot.subsystems.arm.wrist;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.util.Helpers;

public class DummyWrist extends SubsystemBase implements IWrist {
    private final DCMotorSim wristSim;
    private final ProfiledPIDController wristPIDController;
    private Trigger atWantedStateTrigger;

    private static DummyWrist instance;

    private DummyWrist() {
        wristSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.001);
        wristPIDController = new ProfiledPIDController(
                0.5,
                0,
                0,
                new TrapezoidProfile.Constraints(
                        0.5,
                        0.5
                )
        );
        atWantedStateTrigger = new Trigger(this::atState);
    }

    private void setWrist(GamePiece gamePiece) {
        wristPIDController.setGoal(switch (gamePiece) {
            case CONE -> ArmConstants.CONE_ANGLE;
            case CUBE -> ArmConstants.CUBE_ANGLE;
        });
    }

    private void runWrist() {
        double motorVoltage = wristPIDController.calculate(
                wristSim.getAngularPositionRotations()
        );
        wristSim.setInputVoltage(motorVoltage);
        wristSim.update(0.020);
    }

    private boolean atState() {
        return Helpers.withinTolerance(
                wristPIDController.getGoal().position,
                wristSim.getAngularPositionRotations(),
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

    public static DummyWrist getInstance() {
        if (instance == null) {
            instance = new DummyWrist();
        }
        return instance;
    }
}
