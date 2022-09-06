package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class XboxDrive extends CommandBase {
    public static XboxController xboxController;
    public static Tank tank;

    public XboxDrive(XboxController xboxController, Tank tank){
        this.xboxController=xboxController;
        this.tank=tank;
        addRequirements(tank);
    }

    @Override
    public void execute() {
        tank.drive(xboxController.getLeftX(), xboxController.getRightY());
    }

    @Override
    public void end(boolean interrupted) {
        Tank.mainRightMotor.stopMotor();
        Tank.mainLeftMotor.stopMotor();
    }
}
