package frc.robot.subsystems.gripper.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class PowerControlWeee extends CommandBase {
    public Gripper gripper;
    public XboxController xbox;

    public double power;

    public PowerControlWeee(double power){
        this.power = power;
    }

    public PowerControlWeee(XboxController xbox) {
        this.xbox = xbox;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setWeeeMotorPower(power);
    }
}
