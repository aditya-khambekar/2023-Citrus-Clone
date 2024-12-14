package frc.robot.subsystems.arm.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.GameConstants.GamePiece;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.arm.constants.ArmConstants.ArmSuperstructureState;
import frc.robot.subsystems.arm.constants.ArmPIDs;
import frc.robot.util.Helpers;

public class ConcretePivotSubsystem extends PivotSubsystem {
    private final CANSparkMax leftPivot, rightPivot;
    private final DutyCycleEncoder encoder;
    private final ProfiledPIDController pivotPID;

    public ConcretePivotSubsystem() {
        leftPivot = new CANSparkMax(
                ArmConstants.IDs.LEFT_PIVOT_ID,
                CANSparkLowLevel.MotorType.kBrushless
        );
        leftPivot.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightPivot = new CANSparkMax(
                ArmConstants.IDs.RIGHT_PIVOT_ID,
                CANSparkLowLevel.MotorType.kBrushless
        );
        rightPivot.follow(leftPivot, true);
        rightPivot.setIdleMode(CANSparkBase.IdleMode.kBrake);

        encoder = new DutyCycleEncoder(
                new DigitalInput(ArmConstants.IDs.PIVOT_ENCODER_ID)
        );
        encoder.setPositionOffset(0.5);
        pivotPID = new ProfiledPIDController(
                ArmPIDs.pivotKp.get(),
                ArmPIDs.pivotKi.get(),
                ArmPIDs.pivotKd.get(),
                new TrapezoidProfile.Constraints(
                        ArmPIDs.pivotVelocity.get(),
                        ArmPIDs.pivotAcceleration.get()
                )
        );
    }

    public void setPivot(ArmSuperstructureState state, GamePiece gamePiece) {
        pivotPID.setGoal(
                switch (state) {
                    case IDLE -> ArmConstants.PivotConstants.DOWN_ANGLE;
                    case GROUND_INTAKING -> ArmConstants.PivotConstants.GROUND_INTAKING_ANGLE;
                    case SUBSTATION_INTAKING -> ArmConstants.PivotConstants.SUBSTATION_INTAKING_ANGLE;
                    default -> gamePiece == GamePiece.CONE ?
                            ArmConstants.PivotConstants.CONE_PIVOT_ANGLE :
                            ArmConstants.PivotConstants.CUBE_PIVOT_ANGLE;
                });
    }

    public void runPivot() {
//        only uncomment after setting upper and lower values
        leftPivot.set(pivotPID.calculate(
                encoder.get()
        ));
    }

    protected boolean atState() {
        return Helpers.withinTolerance(
                pivotPID.getGoal().position,
                encoder.get(),
                ArmConstants.PivotConstants.PIVOT_TOLERANCE);
    }

    public double getCurrentRotation() {
        return encoder.get();
    }

    public double getTargetRotation() {
        return pivotPID.getGoal().position;
    }

    @Override
    public void periodic() {
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
        runPivot();
        SmartDashboard.putNumber("Pivot output", leftPivot.get());
        SmartDashboard.putNumber("Pivot PID output", pivotPID.calculate(encoder.get()));
        SmartDashboard.putNumber("Pivot Target Angle", pivotPID.getGoal().position);
    }
}
