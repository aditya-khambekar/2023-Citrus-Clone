package frc.robot.subsystems.arm.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.GameConstants.*;
import frc.robot.subsystems.arm.constants.ArmConstants.*;

import java.util.Objects;

public abstract class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;

    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
//            instance = Robot.isReal() ? new ConcreteElevatorSubsystem() : new SimElevatorSubsystem();
            instance = new SimElevatorSubsystem();
        }
        return instance;
    }

    public abstract void setElevator(ArmSuperstructureState state, GamePiece gamePiece);

    public abstract void runElevator();

    protected abstract boolean atState();

    public abstract double getElevatorPosition();

    public Trigger atRequestedStateTrigger() {
        return new Trigger(this::atState);
    }
}
