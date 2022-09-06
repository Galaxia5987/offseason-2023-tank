package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.UnitModel;

public class Tank extends SubsystemBase {
    public static WPI_TalonFX mainLeftMotor = new WPI_TalonFX(Ports.MOTOR);
    public static WPI_TalonFX auxLeftMotor = new WPI_TalonFX(Ports.MOTOR);
    public static WPI_TalonFX mainRightMotor = new WPI_TalonFX(Ports.MOTOR);
    public static WPI_TalonFX auxRightMotor =  new WPI_TalonFX(Ports.MOTOR);
    public static Tank INSTANCE = null;
    public static UnitModel unitModel = new UnitModel(Constants.TICKS_PER_M);
    public Tank(){
        auxLeftMotor.follow(mainLeftMotor);
        auxRightMotor.follow(mainRightMotor);
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
    }

    public Tank getINSTANCE(){
        if (INSTANCE==null){
            INSTANCE=new Tank();
            return INSTANCE;
        }
        else{
            return INSTANCE;
        }
    }

    public double deadBend(double value){
        if (value>=Constants.DEAD_BEND||value<=Constants.DEAD_BEND){
            return 0;
        }
        else{
            return value;
        }
    }

    public void drive(double forwardsPower, double rotationPower){
        mainRightMotor.set(forwardsPower-1*rotationPower);
        mainLeftMotor.set(forwardsPower+1*rotationPower);

    }

    public double getLeftWheelVelocity(){
        mainLeftMotor.
    }
}
