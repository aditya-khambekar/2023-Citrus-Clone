package frc.robot.constants;

public class Constants {
    public static final double DEADZONE_VALUE = 0.08;

    public static final boolean tuningMode = true;
    public enum SysIdMode {
        SWERVE,
        PIVOT,
        ELEVATOR
    }
    public static SysIdMode sysIdMode = null;
}
