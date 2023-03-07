package frc.robot.subsystems.tank;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.UnitModel;

public class Tank extends SubsystemBase {
    private final WPI_TalonFX mainLeftMotor = new WPI_TalonFX(Ports.MAIN_LEFT_MOTOR);
    private final WPI_TalonFX auxLeftMotor = new WPI_TalonFX(Ports.AUX_LEFT_MOTOR);
    private final WPI_TalonFX mainRightMotor = new WPI_TalonFX(Ports.MAIN_RIGHT_MOTOR);
    private final WPI_TalonFX auxRightMotor = new WPI_TalonFX(Ports.AUX_RIGHT_MOTOR);
    private static Tank INSTANCE = null;
    private final UnitModel unitModel = new UnitModel(Constants.TICKS_PER_M);

    private Tank() {
        mainLeftMotor.setNeutralMode(NeutralMode.Coast);
        mainRightMotor.setNeutralMode(NeutralMode.Coast);
        mainLeftMotor.enableVoltageCompensation(Constants.ENABLE_VOLT_COMP);
        mainRightMotor.configVoltageCompSaturation(Constants.CONFIG_VOLT_COMP_SATURATION);
        mainLeftMotor.setInverted(Constants.CLOCKWISE);
        mainRightMotor.setInverted(Constants.COUNTER_CLOCKWISE);
        mainLeftMotor.config_kP(0, Constants.KP, Constants.TALON_TIME_OUT);
        mainLeftMotor.config_kP(0, Constants.KI, Constants.TALON_TIME_OUT);
        mainLeftMotor.config_kP(0, Constants.KD, Constants.TALON_TIME_OUT);
        mainRightMotor.config_kP(0, Constants.KP, Constants.TALON_TIME_OUT);
        mainRightMotor.config_kP(0, Constants.KI, Constants.TALON_TIME_OUT);
        mainRightMotor.config_kP(0, Constants.KD, Constants.TALON_TIME_OUT);
        auxLeftMotor.follow(mainLeftMotor);
        auxLeftMotor.setInverted(Constants.CLOCKWISE);
        auxRightMotor.follow(mainRightMotor);
        auxRightMotor.setInverted(Constants.COUNTER_CLOCKWISE);
    }


    /**
     * if the tank doesn't have an instance it creates one
     *
     * @return the tank instance
     */
    public static Tank getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new Tank();
        }
        return INSTANCE;
    }

    /**
     * lets the tank drive and rotate
     *
     * @param forwardsPower
     * @param rotationPower
     */
    public void drive(double forwardsPower, double rotationPower) {
        mainRightMotor.set(forwardsPower - 1 * rotationPower);
        mainLeftMotor.set(forwardsPower + 1 * rotationPower);
    }
}
