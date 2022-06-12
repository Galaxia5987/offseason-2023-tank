package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DefaultDrive extends CommandBase {
    private final Drivetrain drive;
    private final XboxController controller;

    public DefaultDrive(XboxController controller) {
        this.controller = controller;
        drive = Drivetrain.getInstance();
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(-controller.getLeftY(), controller.getRightX(), Constants.Drivetrain.DEFAULT_INPUT_DEADBAND);
    }
}
