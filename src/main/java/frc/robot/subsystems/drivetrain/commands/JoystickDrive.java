package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class JoystickDrive extends CommandBase {
    private final Drivetrain drive;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;


    public JoystickDrive(Joystick rightJoystick, Joystick leftJoystick) {
        this.rightJoystick = rightJoystick;
        this.leftJoystick = leftJoystick;
        drive = Drivetrain.getInstance();
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drivePower(-leftJoystick.getY(), rightJoystick.getX(), Constants.Drivetrain.DEFAULT_INPUT_DEADBAND);
    }
}
