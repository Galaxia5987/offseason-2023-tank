package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Tank extends SubsystemBase {
    public static WPI_TalonFX mainLeftMotor = new WPI_TalonFX(Ports.Main_Left_Motor);
    public static WPI_TalonFX auxLeftMotor = new WPI_TalonFX(Ports.Aux_Left_Motor);
    public static WPI_TalonFX mainRightMotor = new WPI_TalonFX(Ports.Main_Right_Motor);
    public static WPI_TalonFX auxRightMotor = new WPI_TalonFX(Ports.Aux_Right_Mototr);
    public static Tank INSTANCE = null;
    public static UnitModel unitModel = new UnitModel(Constants.Ticks_Per_Meter);

    private Tank() {
        auxLeftMotor.follow(mainLeftMotor);
        auxRightMotor.follow(mainRightMotor);
        mainLeftMotor.setNeutralMode(NeutralMode.Coast);
    }

    public  double getLeftWheelPower() {
        return mainLeftMotor.get();
    }
    public void setLeftWheelPower(double forward, double rotation) {
        mainRightMotor.set(forward - 1 * rotation);
    }

    public  void setRightWheelPower(double forward, double rotation) {
        mainRightMotor.set(forward - 1 * rotation);
    }
    public double getRightWheelPower(){
        return mainRightMotor.get();

    }
    public double deadband(double value){
        if (value > 0.05 || value < -0.05){
            return value;
        }
 else return 0;
    }

}


