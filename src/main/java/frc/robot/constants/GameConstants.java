package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GameConstants {
    public enum GamePiece{
        CONE,
        CUBE
    }

    public static final double SUBSTATION_X = 15.24;
    public static final double LEFT_SUBSTATION_Y = 7.36;
    public static final double RIGHT_SUBSTATION_Y = 6.14;
    public static final Pose2d LEFT_SUBSTATION_POSE = new Pose2d(
            SUBSTATION_X,
            LEFT_SUBSTATION_Y,
            new Rotation2d(0));
    public static final Pose2d RIGHT_SUBSTATION_POSE = new Pose2d(
            SUBSTATION_X,
            RIGHT_SUBSTATION_Y,
            new Rotation2d(0));
}
