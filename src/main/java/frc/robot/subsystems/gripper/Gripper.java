package frc.robot.subsystems.gripper;

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

}
