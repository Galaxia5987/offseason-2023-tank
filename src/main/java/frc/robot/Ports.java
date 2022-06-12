package frc.robot;

public final class Ports {
    public static class Elevator {
        public static final boolean SENSOR_PHASE = false;
        public static final int SENSOR_POS = 0; // Whether the sensor is inverted.
        public static final int ELE_MOTOR = 13; // The port of the motor.
        public static final int PID_X = 0; // The slot of the PID.
    }

    public static final class Drivetrain {
        public static final int RIGHT_MASTER = 0;
        public static final int RIGHT_SLAVE = 0;

        public static final int LEFT_MASTER = 0;
        public static final int LEFT_SLAVE = 0;
    }

    public static class Controls {
        public static final int XBOX = 0;
    }

}
