package frc.robot.subsystems.elevator;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Elevator.*;


public class Elevator extends SubsystemBase {
    public static final CANSparkMax motor = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
    DCMotor armGearbox = DCMotor.getNEO(2);
    private final ElevatorSim elevatorSim = new ElevatorSim(armGearbox, GEAR_RATIO, mass, DRUM_RADIUS, 0, MAX_HEIGHT);

    Mechanism2d Mech2d = new Mechanism2d(3, 3);
    MechanismRoot2d elevatorRoot2d = Mech2d.getRoot("elevator", 2, 0);
    MechanismLigament2d elevatorMech2d = elevatorRoot2d.append(
            new MechanismLigament2d("elevator", elevatorSim.getPositionMeters(), 90)
    );

    private final PIDController PIDController = new PIDController(kP, 0, 0);

    public static final Encoder encoder = new Encoder(0, 1);
    private static Elevator INSTANCE = null;

    private static JoystickButton leftStick = new JoystickButton(new XboxController(Ports.Controls.XBOX_ELEVATOR), XboxController.Button.kLeftStick.value);
    private final UnitModel unitMan = new UnitModel(TICKS_PER_METER_NEO);

    private final EncoderSim encoderSim = new EncoderSim(encoder);
    private double setpointHeight = 0;

    /**
     * Configure the elevator motor.
     */
    private Elevator() {
        motor.setInverted(INVERTED);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        configurePID();

    }

    public static Elevator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Elevator();
        }
        return INSTANCE;
    }

    private void configurePID() {
        PIDController.setP(0.2);
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
        return motor.get();
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
        setpointHeight = unitMan.toUnits(position);
        encoder.setPosition(position);
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
        motor.set(0);
    }

    /**
     * Gets the motor current.
     *
     * @return the current output of the motor. [A]
     */
//    //public double getCurrentOutput() {
//        return motor.getSupplyCurrent();
//    }

    /**
     * Resets the encoder of the elevator.
     */
//    public void resetEncoder() {
//        motor.setSelectedSensorPosition(0);
//    }
    @Override
    public void periodic() {
        configurePID();
    }

    public void teleopPeriodic() {
        if (leftStick.get()) {
            // Here, we run PID control like normal, with a constant setpoint of 30in.
            double pidOutput = PIDController. (m_encoder.getDistance(), Units.inchesToMeters(30));
            motor.setVoltage(pidOutput);
        } else {
            // Otherwise, we disable the motor.
            motor.set(0.0);
        }
    }

    @Override
    public void simulationPeriodic() {

        elevatorSim.setInput(motor.get() * RobotController.getBatteryVoltage());

        elevatorSim.update(0.020);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
        encoder.setDistance(m_elevatorSim.getPositionMeters());



        elevatorMech2d.setLength(elevatorSim.getPositionMeters());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));


    }
}
