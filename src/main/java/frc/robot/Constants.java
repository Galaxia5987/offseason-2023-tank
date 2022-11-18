package frc.robot;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
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

    public static class Elevator {
        public static final int ACCELERATION = 2; // The acceleration for the trapezoid control mode. [m/s^2]
        public static final int MAX_VELOCITY = 1; // The cruise velocity. [m/s]
        public static final WebConstant STALL_CURRENT = WebConstant.of("Elevator", "Stall Current", 16);

        public static final double kP = 0.2; // Proportional constant.
        public static final double kI = 0; // Integral constant.
        public static final double kD = 0.02; // Derivative constant.
        public static final double MAX_HEIGHT = 1.51; // Maximum height of the elevator. [m]
        public static final double DRUM_RADIUS = 0.03; // Radius of the elevator drum. [m]
        public static final double SLOW_MOVEMENT = MAX_HEIGHT / 5; // Makes the elevator finish moving at 5s. [m/s]
        public static final double TICKS_PER_METER = 2 * Math.PI * DRUM_RADIUS / 42; // [tick]
        public static final double g = 9.80665;
        public static final double G = 1 / 10.0; // gear ratio
        public static final double radius = 0; // [m]
        public static final double mass = 0; // [kg]
        public static final double kF = (Falcon.R * radius * mass * mass * g) / (G * Falcon.Kt); // Takes into account the force that gravity applies (feed forward).
        public static final int ENABLE_VOLT_COMP = 12;

        public static final boolean INVERTED = false; // Whether the motor is inverted.

        public static final Matrix<N2, N1> MODEL_TOLERANCE = Matrix.mat(Nat.N2(), Nat.N1()).fill(
                0.0508,
                1.016
        );
        public static final Matrix<N1, N1> SENSOR_TOLERANCE = Matrix.mat(Nat.N1(), Nat.N1()).fill(
                0.001
        );

        public static final Vector<N2> qelms = VecBuilder.fill(
                0.0254,
                0.254
        );
        public static final Vector<N1> relms = VecBuilder.fill(
                0.3048
        );
    }

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

    public static final class Gripper {
        public static final int TICKS_PER_ROTATION = 1024;
        public static final boolean ENABLE_VOLT_COMP = true;
        public static final int CONFIG_VOLT_COMP = 12;
        public static final InvertType CLOCKWISE = InvertType.InvertMotorOutput;
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
