// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.Controls;
import frc.robot.constants.GameConstants;
import frc.lib.oi.OI;
import frc.robot.subsystems.arm.ArmSuperstructure;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;

public class RobotContainer {
    private ArmSuperstructure arm;
    private final CommandSwerveDrivetrain swerve;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        arm = ArmSuperstructure.getInstance();
        swerve = TunerConstants.DriveTrain;
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Null Auto", Commands.waitSeconds(1));
        autoChooser.addOption("P1", swerve.followChoreoPath("P1", true));
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.driveFieldCentricCommand());
        arm.setDefaultCommand(
                arm.setStateCommand(
                        ArmConstants.ArmSuperstructureState.IDLE,
                        Controls.OperatorControls.getQueuedGamePiece()
                )
        );
        if (Constants.sysIdMode != null) {
            switch (Constants.sysIdMode) {
                case SWERVE:
                    new Trigger(OI.getInstance().driverController()::getXButton).whileTrue(
                            swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                    );
                    new Trigger(OI.getInstance().driverController()::getYButton).whileTrue(
                            swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                    );
                    new Trigger(OI.getInstance().driverController()::getAButton).whileTrue(
                            swerve.sysIdDynamic(SysIdRoutine.Direction.kForward)
                    );
                    new Trigger(OI.getInstance().driverController()::getBButton).whileTrue(
                            swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                    );
                    break;
                case ELEVATOR:
                    new Trigger(OI.getInstance().driverController()::getXButton).onTrue(
                            arm.elevatorQuasistatic(SysIdRoutine.Direction.kForward)
                    );
                    new Trigger(OI.getInstance().driverController()::getYButton).onTrue(
                            arm.elevatorQuasistatic(SysIdRoutine.Direction.kReverse)
                    );
                    new Trigger(OI.getInstance().driverController()::getAButton).onTrue(
                            arm.elevatorDynamic(SysIdRoutine.Direction.kForward)
                    );
                    new Trigger(OI.getInstance().driverController()::getBButton).onTrue(
                            arm.elevatorDynamic(SysIdRoutine.Direction.kReverse)
                    );
                    break;
                case PIVOT:
                    break;
            }
        }
        Controls.DriverControls.leftSubstation.onTrue(
                Commands.sequence(
                        swerve.pathfindCommand(GameConstants.LEFT_SUBSTATION_POSE),
                        Commands.parallel(
                                arm.setStateCommand(
                                        ArmConstants.ArmSuperstructureState.SUBSTATION_INTAKING,
                                        Controls.OperatorControls.getQueuedGamePiece()
                                ),
                                OI.getInstance().driverController().rumbleDurationCommand(.25)
                        ),
                        Commands.waitSeconds(1)
                )
        );
        Controls.DriverControls.rightSubstation.onTrue(
                Commands.sequence(
                        swerve.pathfindCommand(GameConstants.RIGHT_SUBSTATION_POSE),
                        Commands.parallel(
                                arm.setStateCommand(
                                        ArmConstants.ArmSuperstructureState.SUBSTATION_INTAKING,
                                        Controls.OperatorControls.getQueuedGamePiece()
                                ),
                                OI.getInstance().driverController().rumbleDurationCommand(.25)
                        ),
                        Commands.waitSeconds(1)
                )
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void sendSubsystemData() {
        SmartDashboard.putData(swerve);
        SmartDashboard.putData(arm);
    }
}
