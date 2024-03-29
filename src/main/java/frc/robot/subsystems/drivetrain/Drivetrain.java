package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;

/*
 * Drivetrain subsystem class.
 *
 * This class includes drive with both velocity and power, together with the conversions
 * between (right, left) velocities to (forward, rotation).
 */
public class Drivetrain extends SubsystemBase {
    private static Drivetrain INSTANCE = null;
    private final UnitModel unitModel = new UnitModel(Constants.Drivetrain.TICKS_PER_METER);

    private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Ports.Drivetrain.RIGHT_MASTER);
    private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Ports.Drivetrain.LEFT_MASTER);

    private Drivetrain() {
        rightMaster.setNeutralMode(Constants.Drivetrain.NEUTRAL_MODE);
        leftMaster.setNeutralMode(Constants.Drivetrain.NEUTRAL_MODE);

        WPI_TalonSRX rightSlave = new WPI_TalonSRX(Ports.Drivetrain.RIGHT_SLAVE);
        rightSlave.follow(rightMaster);
        WPI_TalonSRX leftSlave = new WPI_TalonSRX(Ports.Drivetrain.LEFT_SLAVE);
        leftSlave.follow(leftMaster);

        rightMaster.setInverted(false);
        leftMaster.setInverted(true);
        rightSlave.setInverted(false);
        leftSlave.setInverted(true);
    }

    public static Drivetrain getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Drivetrain();
        }
        return INSTANCE;
    }

    /**
     * Main drive command.
     * This method is used to actually move the drivetrain.
     *
     * @param forward the forward power. [-1,1]
     * @param rotation the rotation power. [-1,1]
     * @param deadband the interval of power in which the drivetrain doesn't move.
     */
    public void drivePower(double forward, double rotation, double deadband) {
        forward = Utils.continuousDeadband(forward, deadband);
        rotation = Utils.continuousDeadband(rotation, deadband);

        double powerRight = forward - (Constants.Drivetrain.INVERT_ROTATION ? -1 : 1) *
                Constants.Drivetrain.ROTATION_REDUCTION * rotation;
        double powerLeft = forward + (Constants.Drivetrain.INVERT_ROTATION ? -1 : 1) *
                Constants.Drivetrain.ROTATION_REDUCTION * rotation;

        rightMaster.set(ControlMode.PercentOutput, powerRight);
        leftMaster.set(ControlMode.PercentOutput, powerLeft);
    }

    /**
     * Velocity based drive command.
     *
     * @param forward is the velocity forward of the drivetrain. [m/s]
     * @param rotation is the rotational velocity of the drivetrain. [rad/s]
     * @param forwardDeadband is the interval in which the velocity is 0.
     * @param rotationDeadband is the interval in which the rotation is 0.
     */
    public void driveVelocity(double forward, double rotation, double forwardDeadband, double rotationDeadband) {
        forward = Utils.continuousDeadband(forward, forwardDeadband);
        rotation = Utils.continuousDeadband(rotation, rotationDeadband);

        double velocityLeft = (2 * forward - rotation * Math.PI * Constants.Drivetrain.CHASSIS_WIDTH) / 2;
        double velocityRight = 2 * forward - velocityLeft;

        rightMaster.set(ControlMode.Velocity, unitModel.toTicks100ms(velocityRight));
        leftMaster.set(ControlMode.Velocity, unitModel.toTicks100ms(velocityLeft));
    }

    /**
     * Gets the speeds of the two sides of the drivetrain.
     *
     * @return the speeds as differential drive speeds.
     */
    public DifferentialDrive.WheelSpeeds getDrivebaseVelocities() {
        return new DifferentialDrive.WheelSpeeds(
                unitModel.toVelocity(rightMaster.getSelectedSensorVelocity()),
                unitModel.toVelocity(leftMaster.getSelectedSensorVelocity()));
    }

    /**
     * Gets the speeds as forward and rotation power.
     * See class DriveSpeeds below.
     *
     * @return the forward speed. [m/s]
     *         the rotation speed. [rad/s]
     */
    public DriveSpeeds getDriveSpeeds() {
        DifferentialDrive.WheelSpeeds speeds = getDrivebaseVelocities();
        return new DriveSpeeds((speeds.left + speeds.right) / 2,
                (speeds.right - speeds.left) / (Math.PI * Constants.Drivetrain.CHASSIS_WIDTH));
    }

    public static class DriveSpeeds {
        public final double forward; // [m/s]
        public final double rotation; // [rad/s]

        public DriveSpeeds(double forward, double rotation) {
            this.forward = forward;
            this.rotation = rotation;
        }
    }
}
