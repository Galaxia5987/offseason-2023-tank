package frc.robot;


import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public final class Constants {
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].

    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final double NOMINAL_VOLTAGE = 10; // [volts]
    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]
    public static final double SIMULATION_LOOP_PERIOD = 0.02; // [s]

    public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    public static final class Elevator {
        public static final double CURRENT_LIMIT = 30; // [amp]
        public static final double DRUM_RADIUS = 1; // [m]
        public static final double Kp = 1;
        public static final double Ki = 0;
        public static final double Kd = 0;
        public static final double Kf = 0;
        public static final TalonFXInvertType MOTOR_INVERSION = TalonFXInvertType.CounterClockwise;

        public static final double GEAR_RATIO = 0.1; // Ratio between motor and effective gear.
        public static final double TICKS_PER_DEGREE = (2048 / 360.0) / GEAR_RATIO; // Ratio between motor ticks to effective gear degrees.
        public static final double DEGREES_PER_METER = 360 / (2 * Math.PI * DRUM_RADIUS); // Ratio between drum degrees to circumference.
        public static final double TICKS_PER_METER = TICKS_PER_DEGREE * DEGREES_PER_METER; // Ratio between motor ticks to elevator height.
    }
}
