package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class PositionControl extends CommandBase {
    private final Elevator elevator;
    private final double goalHeight;
    private final Timer timer;
    private double lastTime = 0;

    public PositionControl(double height) {
        this.elevator = Elevator.getInstance();
        this.goalHeight = height;
        timer = new Timer();
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double currTime = timer.get();
        elevator.setHeight(goalHeight, currTime - lastTime);
        if (elevator.getSetpointHeight() != goalHeight) {
            DriverStation.reportError("No correlation between desired elevator setpoint and given.", false);
        }
        lastTime = currTime;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.terminate();
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint(goalHeight, 0.05);
    }
}
