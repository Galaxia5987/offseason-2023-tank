package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain INSTANCE = null;
    private final UnitModel unitModel = new UnitModel(Constants.Drivetrain.TICKS_PER_METER);

    private final WPI_TalonFX rightMaster = new WPI_TalonFX(Ports.Drivetrain.RIGHT_MASTER);
    private final WPI_TalonFX leftMaster = new WPI_TalonFX(Ports.Drivetrain.LEFT_MASTER);

    private Drivetrain() {
        rightMaster.setNeutralMode(Constants.Drivetrain.NEUTRAL_MODE);
        leftMaster.setNeutralMode(Constants.Drivetrain.NEUTRAL_MODE);

        WPI_TalonFX rightSlave1 = new WPI_TalonFX(Ports.Drivetrain.RIGHT_SLAVE);
        rightSlave1.follow(rightMaster);
        WPI_TalonFX rightSlave2 = new WPI_TalonFX(Ports.Drivetrain.RIGHT_SLAVE);
        rightSlave2.follow(rightMaster);
        WPI_TalonFX leftSlave1 = new WPI_TalonFX(Ports.Drivetrain.LEFT_SLAVE);
        leftSlave1.follow(leftMaster);
        WPI_TalonFX leftSlave2 = new WPI_TalonFX(Ports.Drivetrain.LEFT_SLAVE);
        leftSlave2.follow(leftMaster);
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
    public void drive(double forward, double rotation, double deadband) {
        forward = Utils.conventionalDeadband(forward, deadband);
        rotation = Utils.conventionalDeadband(rotation, deadband);

        double powerRight = forward - (Constants.Drivetrain.INVERT_ROTATION ? -1 : 1) *
                Constants.Drivetrain.ROTATION_REDUCTION * rotation;
        double powerLeft = forward + (Constants.Drivetrain.INVERT_ROTATION ? -1 : 1) *
                Constants.Drivetrain.ROTATION_REDUCTION * rotation;

        rightMaster.set(ControlMode.PercentOutput, powerRight);
        leftMaster.set(ControlMode.PercentOutput, powerLeft);
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
                (speeds.left - speeds.right) / (Math.PI * Constants.Drivetrain.CHASSIS_WIDTH));
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
