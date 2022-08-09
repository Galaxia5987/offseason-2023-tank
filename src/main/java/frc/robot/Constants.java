package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.valuetuner.WebConstant;

public final class Constants {
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].

    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final double NOMINAL_VOLTAGE = 10; // [volts]
    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]
    public static final double SIMULATION_LOOP_PERIOD = 0.02; // [s]

    public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static class Falcon {
        public static final double NOMINAL_VOLTAGE = 10; // [volt]
        public static final double STALL_TORQUE = 4.69; // [N*m]
        public static final double STALL_CURRENT = 257; // [amps]
        public static final double FREE_CURRENT = 1.5; // [amps]
        public static final double FREE_SPEED = 6380 * Math.PI * 2 / 60.0; // [rad/s]

        public static final double R = NOMINAL_VOLTAGE / STALL_CURRENT; // [ohms]
        public static final double Kv = FREE_SPEED / (NOMINAL_VOLTAGE - R * FREE_CURRENT); // [rad/s*volt]
        public static final double Kt = STALL_TORQUE / STALL_CURRENT; // [N*m/amps]
    }

    public static final class Drivetrain {
        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public static final double ROTATION_REDUCTION = 0.7;
        public static final double CHASSIS_WIDTH = 0.69; // [m]
        public static final boolean INVERT_ROTATION = false;
        public static final double WHEEL_RADIUS = 0.15;
        public static final double TICKS_PER_METER = 2048 / (2 * Math.PI * WHEEL_RADIUS);
        public static final double DEFAULT_INPUT_DEADBAND = 0.05;
    }
}
