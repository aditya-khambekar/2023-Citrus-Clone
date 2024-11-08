package frc.robot.subsystems.drive;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Controls;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.util.CommandsUtil;
import frc.robot.util.DriverStationUtil;
import frc.robot.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem, so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem, Sendable {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.FieldCentricFacingAngle SOTFRequest = new SwerveRequest.FieldCentricFacingAngle();


    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final Field2d field = new Field2d();

    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    null
            ),
            new SysIdRoutine.Mechanism(
                    output -> setControl(translationCharacterization.withVolts(output)),
                    null,
                    this
            )
    );

    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    null
            ),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(steerCharacterization.withVolts(volts)),
                    null,
                    this
            )
    );

    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second),
                    Volts.of(Math.PI),
                    null,
                    null
            ),
            new SysIdRoutine.Mechanism(
                    output -> {
                        setControl(rotationCharacterization.withVolts(output));
                    },
                    null,
                    this
            )
    );

    private final SysIdRoutine sysIdRoutineToApply = sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        seedFieldRelative(new Pose2d());
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SOTFRequest.HeadingController = new PhoenixPIDController(8, 0, 0);
        SOTFRequest.HeadingController.setTolerance(Rotation2d.fromDegrees(7.5).getRadians());
    }

    private SwerveRequest fieldCentricRequestSupplier() {
        double forwardsSpeed = Controls.DriverControls.SwerveForwardAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double sidewaysSpeed = Controls.DriverControls.SwerveStrafeAxis.getAsDouble() * DriveConstants.CURRENT_MAX_ROBOT_MPS;
        double rotationSpeed = Controls.DriverControls.SwerveRotationAxis.getAsDouble() * DriveConstants.TELOP_ROTATION_SPEED;
        return fieldCentricRequest
                .withVelocityX(forwardsSpeed)
                .withVelocityY(sidewaysSpeed)
                .withRotationalRate(rotationSpeed);
    }

    public Command pathfindCommand(Pose2d targetPose) {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        PathConstraints constraints = new PathConstraints(
                10, 5,
                2 * Math.PI, 2 * Math.PI
        );
        return AutoBuilder.pathfindToPoseFlipped(
                targetPose,
                constraints
        );
    }

    public Command driveFieldCentricCommand() {
        return applyRequest(this::fieldCentricRequestSupplier);
    }


    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        field.setRobotPose(getPose());
        SmartDashboard.putData("Drive/Field", field);
        LimelightHelpers.PoseEstimate pose = validatePoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight"), Timer.getFPGATimestamp());
        if (pose != null) {
            addVisionMeasurement(pose.pose, Timer.getFPGATimestamp());
        }
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.020, RobotController.getBatteryVoltage());
        field.setRobotPose(getPose());
        SmartDashboard.putData("Drive/Field", field);
    }

    public void reset() {
        m_pigeon2.setYaw(DriverStationUtil.isRed()?180:0);
    }

    private LimelightHelpers.PoseEstimate validatePoseEstimate(LimelightHelpers.PoseEstimate poseEstimate, double deltaSeconds) {
        if (poseEstimate == null) return null;
        Pose2d pose2d = poseEstimate.pose;
        Translation2d trans = pose2d.getTranslation();
        if (trans.getX() == 0 && trans.getY() == 0) {
            return null;
        }
        return poseEstimate;
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public Rotation2d getRotation2d() {
        return m_pigeon2.getRotation2d();
    }

    public void stop() {
        setControl(new SwerveRequest.SwerveDriveBrake());
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param pathName      The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return followChoreoPath(Choreo.getTrajectory(pathName), resetPosition);
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param trajectory    The Choreo trajectory to follow.
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition) {
        List<Command> commands = new ArrayList<>();

        if (resetPosition) {
            commands.add(runOnce(() -> {
                seedFieldRelative(DriverStationUtil.isRed() ? trajectory.getFlippedInitialPose() : trajectory.getInitialPose());
            }));
        }
        commands.add(choreoSwerveCommand(trajectory));
        return CommandsUtil.sequence(commands);
    }

    // This is a helper method that creates a command that makes the robot follow a Choreo path
    private Command choreoSwerveCommand(ChoreoTrajectory trajectory) {
        return Choreo.choreoSwerveCommand(
                trajectory,
                () -> this.getState().Pose,
                DriveConstants.choreoX,
                DriveConstants.choreoY,
                DriveConstants.choreoRotation,
                (ChassisSpeeds speeds) -> setControl(
                        autoRequest.withSpeeds(speeds)
                ),
                DriverStationUtil::isRed,
                this
        );
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(20, 0, 0),
                        new PIDConstants(20, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> !DriverStationUtil.isRed(),
                this);

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            ArrayList<Trajectory.State> states = new ArrayList<>();
            if (poses.size() > 1) {
                Pose2d lastPose = poses.get(0);
                double t = 0;
                for (var pose : poses.subList(1, poses.size())) {
                    Pose2d delta = new Pose2d(pose.getTranslation().minus(lastPose.getTranslation()), pose.getRotation().minus(lastPose.getRotation()));
                    double curvature = delta.getRotation().getRadians() / delta.getTranslation().getNorm();
                    states.add(new Trajectory.State(t, delta.getX(), delta.getY(), pose, curvature));
                    t += 0.02;
                }
            } else {
                states.add(new Trajectory.State(
                        0,
                        0,
                        0,
                        new Pose2d(-100, -100, new Rotation2d()),
                        0));
            }
            field.getObject("Pathplanner Path").setPoses(poses);
        });
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.setSmartDashboardType("Drive");
        sendableBuilder.addDoubleProperty("Heading",
                () -> getRotation2d().getDegrees(),
                null);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }
}