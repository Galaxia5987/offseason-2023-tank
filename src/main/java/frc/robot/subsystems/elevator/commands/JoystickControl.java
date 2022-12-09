package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class JoystickControl extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();
    private final XboxController xbox;
    public final double power;


    public JoystickControl(XboxController xbox, double power) {
        this.xbox = xbox;
        this.power = power;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (Math.abs(xbox.getLeftY()) > 0.05) {
            elevator.setPower(xbox.getLeftY());
        }

    }
}
