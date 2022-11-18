package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

public class Gripper extends SubsystemBase {
    private static Gripper INSTANCE = null;
    private final UnitModel unitModel = new UnitModel(Constants.Gripper.TICKS_PER_ROTATION);
    private final TalonSRX gripperMotor = new TalonSRX(Ports.Gripper.GRIPPER_MOTOR);
    private final TalonSRX wristMotor = new TalonSRX(Ports.Gripper.WRIST_MOTOR);

    private Gripper() {
        gripperMotor.enableVoltageCompensation(Constants.Gripper.ENABLE_VOLT_COMP);
        gripperMotor.configVoltageCompSaturation(Constants.Gripper.CONFIG_VOLT_COMP);
        gripperMotor.setInverted(Constants.Gripper.CLOCKWISE);//check this
        gripperMotor.setNeutralMode(NeutralMode.Coast);
        wristMotor.enableVoltageCompensation(Constants.Gripper.ENABLE_VOLT_COMP);
        wristMotor.configVoltageCompSaturation(Constants.Gripper.CONFIG_VOLT_COMP);
        wristMotor.setInverted(Constants.Gripper.CLOCKWISE);//check this
        wristMotor.setNeutralMode(NeutralMode.Brake);

    }

    public static Gripper getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new Gripper();
        }
        return INSTANCE;
    }

    public double getWristPower() {
        return wristMotor.getMotorOutputPercent();
    }

    public void setWristPower(double power) {
        wristMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public double getGripperPower() {
        return gripperMotor.getMotorOutputPercent();
    }

    public void setGripperPower(double power) {
        gripperMotor.set(TalonSRXControlMode.PercentOutput, power);
    }


}
