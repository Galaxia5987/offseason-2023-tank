package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain INSTANCE = null;

//    private final WPI_TalonFX rightMaster = new WPI_TalonFX(Ports.Drivetrain.RIGHT_MASTER);
//    private final WPI_TalonFX rightSlave = new WPI_TalonFX(Ports.Drivetrain.RIGHT_SLAVE);
//    private final WPI_TalonFX leftMaster = new WPI_TalonFX(Ports.Drivetrain.LEFT_MASTER);
//    private final WPI_TalonFX leftSlave = new WPI_TalonFX(Ports.Drivetrain.LEFT_SLAVE);

    private Drivetrain() {
    }

    public static Drivetrain getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Drivetrain();
        }
        return INSTANCE;
    }

    public void drive(double forward, double rotation) {

    }
}
