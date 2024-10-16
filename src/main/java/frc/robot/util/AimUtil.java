package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Controls;
import frc.robot.constants.GameConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;

public class AimUtil {
    public static boolean inSubstationRange() {
        Pose2d botPose = TunerConstants.DriveTrain.getPose();
        return botPose.getX() >= GameConstants.SUBSTATION_X - 1;
    }
}