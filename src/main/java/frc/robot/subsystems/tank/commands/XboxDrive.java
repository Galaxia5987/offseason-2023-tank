package frc.robot.subsystems.tank.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tank.Tank;

public class XboxDrive extends CommandBase {
    public static XboxController xboxController;
    public static Tank tank;

    public XboxDrive(XboxController xboxController, Tank tank) {
        this.xboxController = xboxController;
        this.tank = tank;
        addRequirements(tank);
    }

    /**
     * drives the tank with the xbox controller's joysticks' values
     */
    @Override
    public void execute() {
        tank.drive(xboxController.getLeftY(), xboxController.getRightX());
    }

    /**
     * stops the tank
     *
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted) {
        tank.drive(0, 0);
        tank.drive(0, 0);
    }
}
