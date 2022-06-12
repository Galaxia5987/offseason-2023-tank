package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.JoystickControl;
import frc.robot.subsystems.elevator.commands.PositionControl;
import frc.robot.subsystems.elevator.commands.ResetElevator;
import webapp.Webserver;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton midHeight = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton maxHeight = new JoystickButton(xbox, XboxController.Button.kY.value);
    private final JoystickButton resetHeight = new JoystickButton(xbox, XboxController.Button.kX.value);
    private final Elevator elevator = Elevator.getInstance();

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
        elevator.setDefaultCommand(new JoystickControl(() -> -xbox.getLeftY()));
    }

    private void configureButtonBindings() {
        midHeight.whenPressed(new PositionControl(Constants.Elevator.MAX_HEIGHT / 2));
        maxHeight.whenPressed(new PositionControl(Constants.Elevator.MAX_HEIGHT));
        resetHeight.whenPressed(new ResetElevator());
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
