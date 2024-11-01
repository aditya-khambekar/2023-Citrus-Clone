// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Controls;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.arm.ArmSuperstructure;
import frc.robot.subsystems.arm.constants.ArmConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.util.AimUtil;

public class RobotContainer {
    private ArmSuperstructure arm;
    private final CommandSwerveDrivetrain swerve;

    public RobotContainer() {
        arm = ArmSuperstructure.getInstance();
        swerve = TunerConstants.DriveTrain;
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.driveFieldCentricCommand());
//        Controls.DriverControls.leftSubstation.whileTrue(
//                Commands.sequence(
//                        swerve.pathfindCommand(GameConstants.LEFT_SUBSTATION_POSE),
//                        arm.setStateCommand(
//                                ArmConstants.ArmSuperstructureState.SUBSTATION_INTAKING,
//                                Controls.OperatorControls.getQueuedGamePiece()
//                        )
//                )
//        );
//        Controls.DriverControls.rightSubstation.whileTrue(
//                Commands.sequence(
//                        swerve.pathfindCommand(GameConstants.RIGHT_SUBSTATION_POSE),
//                        Commands.parallel(
//                                arm.setStateCommand(
//                                        ArmConstants.ArmSuperstructureState.SUBSTATION_INTAKING,
//                                        Controls.OperatorControls.getQueuedGamePiece()
//                                ).until(arm.atWantedState()))
//                )
//        );
        Controls.DriverControls.leftSubstation.onTrue(
                arm.setStateCommand(ArmConstants.ArmSuperstructureState.SUBSTATION_INTAKING, GameConstants.GamePiece.CUBE)
        );
        Controls.DriverControls.rightSubstation.onTrue(
                arm.setStateCommand(ArmConstants.ArmSuperstructureState.IDLE, GameConstants.GamePiece.CUBE)
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void sendSubsystemData() {
        SmartDashboard.putData(swerve);
        SmartDashboard.putData(arm);
    }
}
