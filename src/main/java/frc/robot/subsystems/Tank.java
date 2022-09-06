package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tank extends SubsystemBase {
    public static WPI_TalonFX mainLeftMotor = new WPI_TalonFX();
    public static WPI_TalonFX auxLeftMotor = new WPI_TalonFX();
    public static WPI_TalonFX mainRightMotor = new WPI_TalonFX();
    public static WPI_TalonFX auxRightMotor =  new WPI_TalonFX();
    public static Tank INSTANCE = null;
    public static UnitModel

    public Tank(){
        auxLeftMotor.follow(mainLeftMotor);
        auxRightMotor.follow(mainRightMotor);
        mainLeftMotor.setNeutralMode(NeutralMode.Coast);
        mainRightMotor.setNeutralMode(NeutralMode.Coast);
    }
}
