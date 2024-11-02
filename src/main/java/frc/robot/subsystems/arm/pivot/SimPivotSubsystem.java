package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.arm.ArmSuperstructure;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmPIDs;

public class SimPivotSubsystem extends PivotSubsystem {
    private double position;
    private final ProfiledPIDController pivotPID;
    private final SingleJointedArmSim pivotSim;

    public SimPivotSubsystem() {
        position = ArmConstants.PivotConstants.DOWN_ANGLE;
        pivotPID = new ProfiledPIDController(
                ArmPIDs.pivotKp.get(),
                ArmPIDs.pivotKi.get(),
                ArmPIDs.pivotKd.get(),
                new TrapezoidProfile.Constraints(
                        ArmPIDs.pivotVelocity.get(),
                        ArmPIDs.pivotAcceleration.get()
                )
        );
        pivotSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        DCMotor.getNEO(2),
                        SingleJointedArmSim.estimateMOI(0.6, 30),
                        200
                ),
                DCMotor.getNEO(2),
                200,
                0.6,
                ArmSuperstructure.getPivotRadians(ArmConstants.PivotConstants.DOWN_ANGLE),
                ArmSuperstructure.getPivotRadians(ArmConstants.PivotConstants.UP_ANGLE),
                true,
                2 * Math.PI * -10 / 360
        );
    }

    @Override
    public void setPivot(ArmConstants.ArmSuperstructureState state, GameConstants.GamePiece gamePiece) {
        position = ArmSuperstructure.getPivotRadians(getTargetPosition(state, gamePiece));
    }

    @Override
    public void periodic() {
        updatePIDs();
        runPivot();
        pivotSim.update(0.020);
        pivotSim.setState(pivotSim.getAngleRads(), pivotSim.getVelocityRadPerSec());
    }

    @Override
    public void runPivot() {
        pivotPID.setGoal(position);
        pivotSim.setInputVoltage(pivotPID.calculate(pivotSim.getAngleRads()));
    }

    @Override
    protected boolean atState() {
        return MathUtil.isNear(position, pivotSim.getAngleRads(), ArmConstants.PivotConstants.PIVOT_TOLERANCE);
    }

    public double getCurrentRotation() {
        return pivotSim.getAngleRads();
    }

    private void updatePIDs() {
        pivotPID.setPID(
                ArmPIDs.pivotKp.get(),
                ArmPIDs.pivotKi.get(),
                ArmPIDs.pivotKd.get()
        );
        pivotPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ArmPIDs.pivotVelocity.get(),
                        ArmPIDs.pivotAcceleration.get()
                )
        );
        SmartDashboard.putNumber("Pivot Raw Angle", pivotSim.getAngleRads());
        SmartDashboard.putNumber("Pivot PID output", pivotPID.calculate(pivotSim.getAngleRads()));
        SmartDashboard.putNumber("Pivot Target Angle", position);
    }
}