package frc.robot.subsystems.gripper.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class MoveWrist extends CommandBase {
    private final Gripper gripper = Gripper.getINSTANCE();
    private final double power;

    public MoveWrist(double power) {
        this.power = power;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
//            gripper.setWristPower(power);
        }


    @Override
    public void end(boolean interrupted) {
//        gripper.setWristPower(0);
    }
}
