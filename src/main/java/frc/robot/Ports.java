package frc.robot;

public final class Ports {
    public static class Elevator {
        public static final boolean SENSOR_PHASE = false;
        public static final int SENSOR_POS = 0; // Whether the sensor is inverted.
        public static final int ELE_MOTOR = 13; // The port of the motor.
        public static final int PID_X = 0; // The slot of the PID.
    }

    public static class Controls {
        public static final int XBOX_ELEVATOR = 0;
    }

}
