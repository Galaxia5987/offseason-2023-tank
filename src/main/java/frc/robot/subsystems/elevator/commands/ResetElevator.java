package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class ResetElevator extends CommandBase {
    private final Elevator elevator;

    public ResetElevator() {
        this.elevator = Elevator.getInstance();
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPower(-0.3);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrentOutput() > Constants.Elevator.STALL_CURRENT.get();
    }
}
