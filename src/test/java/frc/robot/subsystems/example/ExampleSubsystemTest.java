package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.junit.Test;

import static org.junit.Assert.*;

public class ExampleSubsystemTest {

    @Test
    public void testDrivetrain() {
        double forward = 5;
        double rotation = 0;

        double velocityLeft = (2 * forward - rotation * Math.PI * Constants.Drivetrain.CHASSIS_WIDTH) / 2;
        double velocityRight = 2 * forward - velocityLeft;

        System.out.println(velocityLeft + ", " + velocityRight);
        DifferentialDrive.WheelSpeeds wheelSpeeds = new DifferentialDrive.WheelSpeeds(velocityLeft, velocityRight);

        Drivetrain.DriveSpeeds speeds =  new Drivetrain.DriveSpeeds((wheelSpeeds.left + wheelSpeeds.right) / 2,
                (wheelSpeeds.right - wheelSpeeds.left) / (Math.PI * Constants.Drivetrain.CHASSIS_WIDTH));

        System.out.println(speeds.forward + ", " + speeds.rotation);
    }
}