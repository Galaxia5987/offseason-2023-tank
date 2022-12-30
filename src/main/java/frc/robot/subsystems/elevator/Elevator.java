package frc.robot.subsystems.elevator;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;
import frc.robot.valuetuner.WebConstant;

import static frc.robot.Constants.Elevator.*;

/*
 * Elevator subsystem class.
 *
 * This class includes control with two different methods:
 *   1. Position in ticks.
 *   2. Height in meters.
 *
 * Note: There exists a variable saving the current desired setpoint for
 * the elevator. This variable only changes once on of the set functions is called.
 * Note: Resetting the encoder should only be done in cases where the
 * elevator is at the lowest possible height.
 */
public class Elevator extends SubsystemBase {
    public static final CANSparkMax motor = new CANSparkMax(Ports.Elevator.ELE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
//    public static final CANCoder encoder = new CANCoder(Ports.Elevator.ELE_ENCODER);
    public static SparkMaxPIDController PIDController = motor.getPIDController();
    public static RelativeEncoder encoder = motor.getEncoder();
    private static Elevator INSTANCE = null;
    private final UnitModel unitMan = new UnitModel(TICKS_PER_METER);
//    private final WebConstant webKp = WebConstant.of("Elevator", "kP", kP);
//    private final WebConstant webKi = WebConstant.of("Elevator", "kI", kI);
//    private final WebConstant webKd = WebConstant.of("Elevator", "kD", kD);
//    private final WebConstant webKf = WebConstant.of("Elevator", "kF", kF);
    private double setpointHeight = 0;

    /**
     * Configure the elevator motor.
     */
    private Elevator() {
        motor.setInverted(true);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        //motor.configMotionAcceleration(unitMan.toTicks100ms(ACCELERATION));
        motor.enableVoltageCompensation(Constants.Elevator.ENABLE_VOLT_COMP);

        configurePID();
    }

    public static Elevator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Elevator();
        }
        return INSTANCE;
    }

    private void configurePID() {
        PIDController.setP(0.1);
        PIDController.setI(0);
        PIDController.setD(0);
        PIDController.setFF(0);
    }

    /**
     * Gets the position of the motor (used for debugging).
     *
     * @return the position of the motor. [m]
     */
    public double getPosition() {
        return unitMan.toUnits(encoder.getPosition());
    }

    /**
     * Gets the power of the motor.
     *
     * @return the power of the motor. [-1,1]
     */
    public double getPower() {
        return motor.getBusVoltage() * motor.getOutputCurrent();
    }

    /**
     * Sets the power of the motor.
     *
     * @param power the power to set. [-1,1]
     */
    public void setPower(double power) {
        motor.set(power);
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
        double setpointHeightTicks = unitMan.toUnits(position);
        PIDController.setReference(setpointHeightTicks, CANSparkMax.ControlType.kPosition);
//        encoder.setPosition(position);
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
        return Math.abs((setpointIsHeight ? setpoint : unitMan.toTicks(setpoint)) - getHeight()) < tolerance;
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
        motor.set(0);
    }

    /**
     * Gets the motor current.
     *
     * @return the current output of the motor. [A]
     */
    public double getCurrentOutput() {
        return motor.getOutputCurrent();
    }

    /**
     * Resets the encoder of the elevator.
     */
    public void resetEncoder() {
//        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        configurePID();
    }
}
