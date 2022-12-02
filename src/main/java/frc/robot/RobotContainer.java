package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.DefaultDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.JoystickControl;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.commands.PowerControlWeee;
import webapp.Webserver;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final XboxController xbox_drivetrain = new XboxController(Ports.Controls.XBOX_DRIVETRAIN);
    private final XboxController xbox_elevator = new XboxController(Ports.Controls.XBOX_ELEVATOR);
    private final JoystickButton a = new JoystickButton(xbox_drivetrain, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox_drivetrain, XboxController.Button.kB.value);
    private final JoystickButton y = new JoystickButton(xbox_drivetrain, XboxController.Button.kY.value);

    private final Elevator elevator = Elevator.getInstance();
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
        drive.setDefaultCommand(new DefaultDrive(xbox_drivetrain));
        elevator.setDefaultCommand(new JoystickControl(xbox_elevator));
    }

     private void configureButtonBindings() {
a.whileHeld(new PowerControlWeee(0.7));
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
