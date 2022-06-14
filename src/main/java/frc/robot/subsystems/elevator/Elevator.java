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
import frc.robot.valuetuner.WebConstant;

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

    private final WebConstant webKp = WebConstant.of("Elevator", "kP", kP);
    private final WebConstant webKi = WebConstant.of("Elevator", "kP", kI);
    private final WebConstant webKd = WebConstant.of("Elevator", "kP", kD);
    private final WebConstant webKf = WebConstant.of("Elevator", "kP", kF);

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

//        configurePID();

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

    private void configurePID() {
        motor.config_kP(Ports.Elevator.PID_X, webKp.get());
        motor.config_kI(Ports.Elevator.PID_X, webKi.get());
        motor.config_kD(Ports.Elevator.PID_X, webKd.get());
        motor.config_kF(Ports.Elevator.PID_X, webKf.get());
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

    /**
     * Gets the power of the motor.
     *
     * @return the power of the motor. [-1,1]
     */
    public double getPower() {
        return motor.getMotorOutputPercent();
    }

    /**
     * Sets the power of the motor.
     *
     * @param power the power to set. [-1,1]
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    /**
     * Sets the height of the elevator.
     * Converts the height to a position of the motor and calls the position based function.
     *
     * @param height       is the height to move the elevator to. [m]
     * @param timeInterval is the time passed between the last call of the function. [s]
     */
    public void setHeight(double height, double timeInterval) {
        setpointHeight = height;
        setPosition(unitMan.toTicks(height), timeInterval);
    }

    /**
     * Gets the current height of the elevator using the position of the motor.
     *
     * @return the height of the elevator. [m]
     */
    public double getHeight() {
        return unitMan.toUnits(getPosition());
    }

    /**
     * Sets the motor to the desired position.
     *
     * @param position     is the position to set the motor to. [ticks]
     * @param timeInterval is the time interval between this and the last call of the function. [s]
     */
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

    /**
     * Checks whether the elevator is at the last setpoint height.
     *
     * @param setpoint         is the setpoint to check. [m] or [ticks]
     * @param tolerance        is the allowable interval of the setpoint. [m] or [ticks]
     * @param setpointIsHeight whether the setpoint to check is height or motor position.
     * @return whether the elevator is in the desired interval in the desired units.
     */
    public boolean atSetpoint(double setpoint, double tolerance, boolean setpointIsHeight) {
        return Math.abs(setpointIsHeight ? setpoint : unitMan.toTicks(setpoint) - getHeight()) < tolerance;
    }

    /**
     * Gets the current setpoint of the elevator.
     *
     * @return the setpoint height. [m]
     */
    public double getSetpointHeight() {
        return setpointHeight;
    }

    /**
     * Stops the elevator.
     */
    public void terminate() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Gets the motor current.
     *
     * @return the current output of the motor. [A]
     */
    public double getCurrentOutput() {
        return motor.getSupplyCurrent();
    }

    /**
     * Resets the encoder of the elevator.
     */
    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
//        configurePID();
    }
}
