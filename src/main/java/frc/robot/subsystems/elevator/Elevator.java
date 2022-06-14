package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Falcon.*;
import static frc.robot.Constants.LOOP_PERIOD;

public class Elevator extends SubsystemBase {
    public static final TalonFX motor = new TalonFX(Ports.Elevator.ELE_MOTOR);
    private static Elevator INSTANCE = null;
    private final UnitModel unitMan = new UnitModel(TICKS_PER_METER);
    private final LinearSystemLoop<N2, N1, N1> linearSystemLoop;
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ACCELERATION, MAX_VELOCITY);
    private TrapezoidProfile.State lastTrapezoidState;
    private double setpointHeight = 0;

    /**
     * Configure the elevator motor.
     */
    private Elevator() {
        lastTrapezoidState = new TrapezoidProfile.State(getPosition(), 0);

        motor.setSensorPhase(Ports.Elevator.SENSOR_PHASE);
        motor.setSelectedSensorPosition(Ports.Elevator.SENSOR_POS);
        motor.setInverted(INVERTED);

        motor.configMotionAcceleration(unitMan.toTicks100ms(ACCELERATION));
        motor.configMotionCruiseVelocity(unitMan.toTicks100ms(MAX_VELOCITY));

        motor.config_kP(Ports.Elevator.PID_X, kP);
        motor.config_kI(Ports.Elevator.PID_X, kI);
        motor.config_kD(Ports.Elevator.PID_X, kD);

        Matrix<N2, N2> a = Matrix.mat(Nat.N2(), Nat.N2()).fill(
                0,
                0,
                1,
                -(Utils.sqr(G) * Kt) / (R * Utils.sqr(radius) * mass * Kv)
        );

        Matrix<N2, N1> b = Matrix.mat(Nat.N2(), Nat.N1()).fill(
                0,
                G * Kt / (R * radius * mass)
        );

        Matrix<N1, N2> c = Matrix.mat(Nat.N1(), Nat.N2()).fill(
                1,
                0
        );

        Matrix<N1, N1> d = Matrix.mat(Nat.N1(), Nat.N1()).fill(
                0
        );

        LinearSystem<N2, N1, N1> elevatorPlant = new LinearSystem<>(
                a,
                b,
                c,
                d
        );

        KalmanFilter<N2, N1, N1> kalmanFilter = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                elevatorPlant,
                MODEL_TOLERANCE,
                SENSOR_TOLERANCE,
                LOOP_PERIOD
        );

        LinearQuadraticRegulator<N2, N1, N1> lqr = new LinearQuadraticRegulator<>(
                elevatorPlant,
                qelms,
                relms,
                LOOP_PERIOD
        );

        linearSystemLoop = new LinearSystemLoop<>(
                elevatorPlant,
                lqr,
                kalmanFilter,
                NOMINAL_VOLTAGE,
                LOOP_PERIOD
        );
    }

    public static Elevator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Elevator();
        }
        return INSTANCE;
    }

    /**
     * Gets the position of the motor (used for debugging).
     *
     * @return the position of the motor. [m]
     */
    public double getPosition() {
        return unitMan.toUnits(motor.getSelectedSensorPosition());
    }

    public double getPower() {
        return motor.getMotorOutputPercent();
    }

    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void setHeight(double height, double timeInterval) {
        setpointHeight = height;
        setPosition(unitMan.toTicks(height), timeInterval);
    }

    public double getHeight() {
        return unitMan.toUnits(getPosition());
    }

    public void setPosition(double position, double timeInterval) {
        setpointHeight = unitMan.toUnits(position);
        TrapezoidProfile.State goal = new TrapezoidProfile.State(position, 0);
        lastTrapezoidState =
                (new TrapezoidProfile(constraints, goal, lastTrapezoidState)).calculate(LOOP_PERIOD);

        linearSystemLoop.setNextR(lastTrapezoidState.velocity, lastTrapezoidState.position);
        linearSystemLoop.correct(VecBuilder.fill(lastTrapezoidState.position));
        linearSystemLoop.predict(timeInterval);

        double output = linearSystemLoop.getU(0) / NOMINAL_VOLTAGE;
        motor.set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, kF);
    }

    public boolean atSetpoint(double setpoint, double tolerance) {
        return Math.abs(setpoint - getHeight()) < tolerance;
    }

    public double getSetpointHeight() {
        return setpointHeight;
    }

    public void terminate() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    public double getCurrentOutput() {
        return motor.getSupplyCurrent();
    }

    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }
}
