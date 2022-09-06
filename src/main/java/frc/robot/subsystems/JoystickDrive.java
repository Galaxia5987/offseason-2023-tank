package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickDrive extends CommandBase {
    public static Joystick joystick1;
    public static Joystick joystick2;
    public static Tank tank;

    public JoystickDrive(Joystick joystick1, Joystick joystick2, Tank tank){
        this.joystick1=joystick1;
        this.joystick2=joystick2;
        this.tank=tank;
        addRequirements(tank);
    }

    @Override
    public void execute() {
        tank.drive(joystick1.getX(), joystick2.getY());
    }

    @Override
    public void end(boolean interrupted) {
        Tank.mainLeftMotor.stopMotor();
        Tank.mainRightMotor.stopMotor();
    }
}
