package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.DefaultDrive;
import frc.robot.subsystems.drivetrain.commands.JoystickDrive;
import webapp.Webserver;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final Joystick leftJoystick = new Joystick(Ports.Controls.JOYSTICK_LEFT);
    private final Joystick rightJoystick = new Joystick(Ports.Controls.JOYSTICK_RIGHT);
    private final Drivetrain drive = Drivetrain.getInstance();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();

        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }

        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        drive.setDefaultCommand(new DefaultDrive(xbox));
        drive.setDefaultCommand(new JoystickDrive(rightJoystick, leftJoystick));
    }

    private void configureButtonBindings() {
    }


    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
      return null;
    }

    /**
     * Initiates the value tuner.
     */
    private void startValueTuner() {
    }

    /**
     * Initiates the port of team 225s Fire-Logger.
     */
    private void startFireLog() {

        try {
            new Webserver();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
