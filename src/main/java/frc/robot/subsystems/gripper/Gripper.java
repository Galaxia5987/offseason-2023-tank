package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Elevator.TICKS_PER_METER;
import static frc.robot.Constants.Elevator.TICKS_PER_METER_NEO;

public class Gripper extends SubsystemBase {

    public static  final WPI_TalonSRX motor = new WPI_TalonSRX(1);
    private static final Gripper INSTANCE = null;
    private final UnitModel unitMan = new UnitModel(TICKS_PER_METER);


}
