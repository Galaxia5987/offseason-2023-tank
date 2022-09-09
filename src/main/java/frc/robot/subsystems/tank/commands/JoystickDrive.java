package frc.robot.subsystems.tank.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tank.Tank;

public class JoystickDrive extends CommandBase {
    public static Joystick joystick1;
    public static Joystick joystick2;
    public static Tank tank;

    public JoystickDrive(Joystick joystick1, Joystick joystick2, Tank tank) {
        this.joystick1 = joystick1;
        this.joystick2 = joystick2;
        this.tank = tank;
        addRequirements(tank);
    }

    /**
     * drives the tank with the joysticks' values
     */
    @Override
    public void execute() {
        tank.drive(joystick1.getY(), joystick2.getX());
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
