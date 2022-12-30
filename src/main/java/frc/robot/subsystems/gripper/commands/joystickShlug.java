package frc.robot.subsystems.gripper.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class joystickShlug extends CommandBase {
    public Gripper gripper;
    public XboxController xbox;

    double power;

    public joystickShlug(double power){
        this.power = power;
    }

    public joystickShlug(XboxController xbox) {
        this.xbox = xbox;
        addRequirements(gripper);
    }
    public void execute() {
        gripper.setShlugMotor(-xbox.getLeftY());
    }
}
