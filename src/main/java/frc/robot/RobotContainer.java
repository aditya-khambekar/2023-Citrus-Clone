// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Controls;
import frc.robot.constants.GameConstants;
import frc.robot.oi.OI;
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
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
//        swerve.setDefaultCommand(swerve.driveFieldCentricCommand());
//        arm.setDefaultCommand(
//                arm.setStateCommand(
//                        ArmConstants.ArmSuperstructureState.IDLE,
//                        GameConstants.GamePiece.CUBE
//                )
//        );
        Controls.DriverControls.leftSubstation.whileTrue(
                Commands.sequence(
//                        swerve.pathfindCommand(GameConstants.LEFT_SUBSTATION_POSE),
                        arm.setStateCommand(
                                ArmConstants.ArmSuperstructureState.SUBSTATION_INTAKING,
                                Controls.OperatorControls.getQueuedGamePiece()
                        )
                )
        );
        Controls.DriverControls.rightSubstation.whileTrue(
                Commands.sequence(
//                        swerve.pathfindCommand(GameConstants.RIGHT_SUBSTATION_POSE),
                        Commands.parallel(
                                arm.setStateCommand(
//                                        ArmConstants.ArmSuperstructureState.SUBSTATION_INTAKING,
                                        ArmConstants.ArmSuperstructureState.IDLE,
                                        Controls.OperatorControls.getQueuedGamePiece()
                                ).until(arm.atWantedState()))
                )
        );
        new Trigger(OI.getInstance().driverController()::getLeftBumper).whileTrue(
                swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        new Trigger(OI.getInstance().driverController()::getRightBumper).whileTrue(
                swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        new Trigger(OI.getInstance().driverController()::getAButton).whileTrue(
                swerve.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        new Trigger(OI.getInstance().driverController()::getBButton).whileTrue(
                swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );
        NamedCommands.registerCommand(
                "pivot up",
                arm.setStateCommand(ArmConstants.ArmSuperstructureState.SUBSTATION_INTAKING, GameConstants.GamePiece.CUBE)
        );
        NamedCommands.registerCommand(
                "pivot down",
                arm.setStateCommand(ArmConstants.ArmSuperstructureState.IDLE, GameConstants.GamePiece.CUBE)
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
