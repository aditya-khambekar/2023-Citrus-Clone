package frc.robot.subsystems.arm.constants;

public class ArmConstants {
    public static final double DOWN_POSITION = 0;
    public static final double UP_POSITION = 20;
    public static final double DOWN_ANGLE = -1.0/12.0;
    public static final double UP_ANGLE = 1.0/4.0;
    public static final double[] CUBE_POSITIONS = new double[]{0, 8, 16};
    public static final double[] CONE_POSITIONS = new double[]{5, 12, 19};
    public static final double INTAKING_POSITION = 5;
    public static final double CUBE_ANGLE = 1.0/8.0;
    public static final double CONE_ANGLE = 1.0/9.0;
    public static final double CUBE_WRIST_ANGLE = 0;
    public static final double CONE_WRIST_ANGLE = -1.0/4.0;
    public static final double IDLE_WRIST_ANGLE = 1.0/12.0;
    public static final double PIVOT_TOLERANCE = 0.002;
    public static final double ELEVATOR_TOLERANCE = 0.002;
    public static final double WRIST_TOLERANCE = 0.002;

    public static final double PIVOT_SENSOR_TO_MECHANISM_RATIO = 1;
    public static final double ELEVATOR_ROTOR_TO_SENSOR_RATIO = 1;
    public static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = 1;

    public static final double INTAKE_SPEED = 0.5;

    public enum ArmSuperstructureState {
        IDLE,
        LOW,
        MID,
        HIGH,
        INTAKING
    }

    public static class IDs {
        public static final int LEFT_PIVOT_ID = 13;
        public static final int RIGHT_PIVOT_ID = 14;

        public static final int LEFT_ELEVATOR_ID = 15;
        public static final int RIGHT_ELEVATOR_ID = 16;

        public static final int PIVOT_ENCODER_ID = 17;
        public static final int ELEVATOR_ENCODER_ID = 18;

        public static final int WRIST_ID = 19;
        public static final int INTAKE_ID = 20;
    }
}
