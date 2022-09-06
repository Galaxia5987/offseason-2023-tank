package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnitModel;

public class Tank extends SubsystemBase {
    public static WPI_TalonFX mainLeftMotor = new WPI_TalonFX();
    public static WPI_TalonFX auxLeftMotor = new WPI_TalonFX();
    public static WPI_TalonFX mainRightMotor = new WPI_TalonFX();
    public static WPI_TalonFX auxRightMotor =  new WPI_TalonFX();
    public static Tank INSTANCE = null;
    public static UnitModel unitModel = new UnitModel();
    public Tank(){
        auxLeftMotor.follow(mainLeftMotor);
        auxRightMotor.follow(mainRightMotor);
        mainLeftMotor.setNeutralMode(NeutralMode.Coast);
        mainRightMotor.setNeutralMode(NeutralMode.Coast);
        mainLeftMotor.enableVoltageCompensation(Constants.ENABLE_VOLT_COMP);
        mainRightMotor.configVoltageCompSaturation(Constants.CONFIG_VOLT_COMPSATURATION);
        mainLeftMotor.setInverted();
        mainRightMotor.setInverted();
    }
}
