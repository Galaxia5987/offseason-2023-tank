package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Elevator.TICKS_PER_METER;
import static frc.robot.Constants.Elevator.TICKS_PER_METER_NEO;

public class Gripper extends SubsystemBase {

    public static  final WPI_TalonSRX shlugMotor = new WPI_TalonSRX(1);
    public static  final WPI_TalonSRX weeeMotor = new WPI_TalonSRX(1);
    private static final Gripper INSTANCE = null;
    private final UnitModel unitMan = new UnitModel(TICKS_PER_METER);

private Gripper() {

}

public void setShlugMotor(double power){
    shlugMotor.set(ControlMode.PercentOutput, power);
}
public void setWeeeMotorAngle(double angle){
    weeeMotor.set(ControlMode.Position, Math.toRadians(angle)* Constants.Gripper.TICKS_PER_RADIAN); //TODO: check in which measurements Cim position works


}
public void setWeeeMotorPower(double power){
    weeeMotor.set(TalonSRXControlMode.PercentOutput, power);
}

public double getShlugMotor(){
    return shlugMotor.get();
}
public double getWeeeMotorPosition(){
    return weeeMotor.get();
}

}
