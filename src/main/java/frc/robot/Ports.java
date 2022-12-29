package frc.robot;

public final class Ports {

    public static final class Drivetrain {
        public static final int RIGHT_MASTER = 40;
        public static final int RIGHT_SLAVE = 11;

        public static final int LEFT_MASTER = 28;
        public static final int LEFT_SLAVE = 22;
    }

    public static final class Elevator {
        public static final int ELE_MOTOR = 20;
        public static final int ELE_ENCODER = 0;
    }

    public static final class Gripper {
        public static final int GRIPPER_MOTOR = 0;
        public static final int WRIST_MOTOR = 0;
    }

    public static class Controls {
        public static final int XBOX_DRIVETRAIN = 0;
        public static final int XBOX_ELEVATOR = 1;
    }

}
