package frc.robot.subsystems.arm.constants;

public class ArmConstants {
    public static final double UP_POSITION = 1;
    public static final double DOWN_POSITION = 0;
    public static final double DOWN_ANGLE = -1.0/12.0;
    public static final double[] CUBE_POSITIONS = new double[]{
            0 * UP_POSITION,
            40 * UP_POSITION,
            80 * UP_POSITION
    };
    public static final double[] CONE_POSITIONS = new double[]{
            25 * UP_POSITION,
            60 * UP_POSITION,
            95 * UP_POSITION
    };
    public static final double GROUND_INTAKING_POSITION = 25 * UP_POSITION;
    public static final double GROUND_INTAKING_ANGLE = -1.0/12.0;
    public static final double CUBE_PIVOT_ANGLE = 1.0/8.0;
    public static final double CONE_PIVOT_ANGLE = 1.0/9.0;
    public static final double SUBSTATION_INTAKING_POSITION = 80 * UP_POSITION;
    public static final double SUBSTATION_INTAKING_ANGLE = 1.0/6.0;
    public static final double CUBE_WRIST_ANGLE = 0;
    public static final double CONE_WRIST_ANGLE = -1.0/4.0;
    public static final double GROUND_INTAKING_CUBE_WRIST_ANGLE = 0;
    public static final double GROUND_INTAKING_CONE_WRIST_ANGLE = -1.0/4.0;
    public static final double SUBSTATION_INTAKING_CUBE_WRIST_ANGLE = -1.0/12.0;
    public static final double SUBSTATION_INTAKING_CONE_WRIST_ANGLE = 1.0/3.0;
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
        GROUND_INTAKING,
        SUBSTATION_INTAKING,
        OUTTAKING
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
