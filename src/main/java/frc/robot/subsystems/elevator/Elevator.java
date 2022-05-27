package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;
import frc.robot.valuetuner.WebConstant;

public class Elevator extends SubsystemBase {
    private static Elevator INSTANCE = null;
    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Elevator.MOTOR);
    private final UnitModel unitModel = new UnitModel(Constants.Elevator.TICKS_PER_METER);

    private final WebConstant webKp = WebConstant.of("Elevator", "kP", Constants.Elevator.Kp);
    private final WebConstant webKi = WebConstant.of("Elevator", "kP", Constants.Elevator.Ki);
    private final WebConstant webKd = WebConstant.of("Elevator", "kP", Constants.Elevator.Kd);
    private final WebConstant webKf = WebConstant.of("Elevator", "kP", Constants.Elevator.Kf);

    private Elevator() {
        motor.config_kP(0, Constants.Elevator.Kp);
        motor.config_kI(0, Constants.Elevator.Ki);
        motor.config_kD(0, Constants.Elevator.Kd);
        motor.config_kF(0, Constants.Elevator.Kf);

        motor.setInverted(Constants.Elevator.MOTOR_INVERSION);
        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.Elevator.CURRENT_LIMIT, 0, 0));
    }

    public static Elevator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Elevator();
        }
        return INSTANCE;
    }

    /**
     * Sets the velocity of the motor.
     *
     * @param velocity the setpoint. [m/s]
     */
    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, unitModel.toTicks100ms(velocity));
    }

    /**
     * Gets the current velocity of the elevator.
     *
     * @return the velocity of the elevator. [m/s]
     */
    public double getVelocity() {
        return unitModel.toVelocity(motor.getSelectedSensorVelocity());
    }

    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public double getPower() {
        return motor.get();
    }

    @Override
    public void periodic() {
        motor.config_kP(0, webKp.get());
        motor.config_kI(0, webKi.get());
        motor.config_kD(0, webKd.get());
        motor.config_kF(0, webKf.get());
    }
}
