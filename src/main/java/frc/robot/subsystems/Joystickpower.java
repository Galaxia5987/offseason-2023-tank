package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Joystickpower extends CommandBase {
    private Tank tank;
    private Joystick leftJoystick;
    private Joystick rightJoystick;

    public Joystickpower(Tank tank, Joystick leftJoystick, Joystick rightJoystick) {
        this.tank = tank;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }
}
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}