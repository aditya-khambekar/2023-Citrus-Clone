package frc.robot.subsystems.elevator.constants;

public class ElevatorConstants {
    public static final double DOWN_POSITION = 0;
    public static final double UP_POSITION = 20;
    public static final double DOWN_ANGLE = 0;
    public static final double UP_ANGLE = 120;
    public static final double[] CUBE_POSITIONS = new double[]{0, 8, 16};
    public static final double[] CONE_POSITIONS = new double[]{5, 12, 19};
    public static final double INTAKING_POSITION = 5;
    public static final double CUBE_ANGLE = 75;
    public static final double CONE_ANGLE = 70;

    public enum ElevatorSuperstructureState{
        IDLE,
        CONE_LOW,
        CONE_MID,
        CONE_HIGH,
        CUBE_LOW,
        CUBE_MID,
        CUBE_HIGH,
        INTAKING
    }

    public static final double ROTOR_TO_SENSOR_RATIO = 0.5;
    public static final double SENSOR_TO_MECHANISM_RATIO = 1;

    public static final int LEFT_PIVOT_ID = 13;
    public static final int RIGHT_PIVOT_ID = 14;

    public static final int LEFT_ELEVATOR_ID = 15;
    public static final int RIGHT_ELEVATOR_ID = 16;

    public static final int ENCODER_ID = 17;
}
