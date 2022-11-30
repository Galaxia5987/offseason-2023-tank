package frc.robot.subsystems.gripper.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class PositionWeeeMotor extends CommandBase {
    public Gripper gripper;

    @Override
    public void initialize() {
        gripper.setWeeeMotor(30);
    }
}
