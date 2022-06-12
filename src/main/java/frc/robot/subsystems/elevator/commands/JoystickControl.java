package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

import java.util.function.DoubleSupplier;

public class JoystickControl extends CommandBase {
    private final Elevator elevator;
    private final DoubleSupplier joystickInput;

    public JoystickControl(DoubleSupplier joystickInput) {
        this.elevator = Elevator.getInstance();
        this.joystickInput = joystickInput;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPower(joystickInput.getAsDouble());
    }
}
