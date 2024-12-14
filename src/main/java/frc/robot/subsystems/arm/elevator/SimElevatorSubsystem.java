package frc.robot.subsystems.arm.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.arm.ArmSuperstructure;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmPIDs;
import frc.robot.util.Helpers;

public class SimElevatorSubsystem extends ElevatorSubsystem {
    private double position;
    private final ProfiledPIDController elevatorPID;
    private final ElevatorSim elevatorSim;

    public SimElevatorSubsystem() {
        elevatorPID = new ProfiledPIDController(
                ArmPIDs.elevatorKp.get(),
                ArmPIDs.elevatorKi.get(),
                ArmPIDs.elevatorKd.get(),
                new TrapezoidProfile.Constraints(
                        ArmPIDs.elevatorVelocity.get(),
                        ArmPIDs.elevatorAcceleration.get()
                )
        );
        elevatorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        DCMotor.getKrakenX60(2),
                        10,
                        0.6,
                        100
                ),
                DCMotor.getKrakenX60(2),
                ArmSuperstructure.getElevatorLength(ArmConstants.ElevatorConstants.DOWN_POSITION),
                ArmSuperstructure.getElevatorLength(ArmConstants.ElevatorConstants.UP_POSITION),
                true,
                ArmSuperstructure.getElevatorLength(ArmConstants.ElevatorConstants.DOWN_POSITION)
        );
        setElevator(ArmConstants.ArmSuperstructureState.IDLE, GameConstants.GamePiece.CONE);
    }

    @Override
    public void setElevator(ArmConstants.ArmSuperstructureState state, GameConstants.GamePiece gamePiece) {
        position = ArmSuperstructure.getElevatorLength(getElevatorPosition(state, gamePiece));
    }

    @Override
    public void runElevator() {
        elevatorPID.setGoal(ArmSuperstructure.getElevatorSetpoint(position));
        double output = elevatorPID.calculate(ArmSuperstructure.getElevatorSetpoint(getCurrentPosition()));
        elevatorSim.setInput(output);
        SmartDashboard.putNumber("Elevator PID output", output);
    }

    @Override
    protected boolean atState() {
        return Helpers.withinTolerance(position, getCurrentPosition(), ArmConstants.ElevatorConstants.ELEVATOR_TOLERANCE);
    }

    @Override
    public double getCurrentPosition() {
        return elevatorSim.getPositionMeters();
    }

    @Override
    public void periodic() {
        updatePIDs();
        runElevator();
        elevatorSim.update(0.020);
        elevatorSim.setState(elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
    }

    private void updatePIDs() {
        elevatorPID.setPID(
                ArmPIDs.pivotKp.get(),
                ArmPIDs.pivotKi.get(),
                ArmPIDs.pivotKd.get()
        );
        elevatorPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ArmPIDs.pivotVelocity.get(),
                        ArmPIDs.pivotAcceleration.get()
                )
        );
    }

    @Override
    public double getTargetPosition() {
        return position;
    }

    public Command quasistatic(SysIdRoutine.Direction direction) {
        return new WaitCommand(0.5);
    }

    public Command dynamic(SysIdRoutine.Direction direction) {
        return quasistatic(direction);
    }
}
