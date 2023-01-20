package frc.robot.subsystems.elevator;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Elevator.*;


public class Elevator extends SubsystemBase {
    public static final CANSparkMax motor = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
    DCMotor armGearbox = DCMotor.getNEO(2);
    private final ElevatorSim elevatorSim = new ElevatorSim(armGearbox, G, mass, DRUM_RADIUS, 0, MAX_HEIGHT, true);

    Mechanism2d Mech2d = new Mechanism2d(3, 3);
    MechanismRoot2d elevatorRoot2d = Mech2d.getRoot("elevator", 2, 0);
    MechanismLigament2d elevatorMech2d = elevatorRoot2d.append(
            new MechanismLigament2d("elevator", elevatorSim.getPositionMeters(), 90)
    );

    private final PIDController PIDController = new PIDController(kP, 0, 0);

    public static final Encoder encoder = new Encoder(0, 1);
    private static Elevator INSTANCE = null;

    private static JoystickButton leftStick = new JoystickButton(new XboxController(Ports.Controls.XBOX_ELEVATOR), XboxController.Button.kLeftStick.value);
    private final UnitModel unitMan = new UnitModel(TICKS_PER_METER_NEO);

    private final EncoderSim encoderSim = new EncoderSim(encoder);
    private double setpointHeight = 0;

    /**
     * Configure the elevator motor.
     */
    private Elevator() {
        motor.setInverted(INVERTED);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        configurePID();

    }

    public static Elevator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Elevator();
        }
        return INSTANCE;
    }

    private void configurePID() {
        PIDController.setP(0.2);
        PIDController.setI(0);
        PIDController.setD(0);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public void periodic() {
        configurePID();
        if (leftStick.getAsBoolean()) {
            double pidOutput = PIDController.calculate(encoder.getDistance(), Units.inchesToMeters(30));
            motor.setVoltage(pidOutput);
        } else {
            // Otherwise, we disable the motor.
            motor.set(0.0);
        }
    }


    @Override
    public void simulationPeriodic() {

        elevatorSim.setInput(motor.get() * RobotController.getBatteryVoltage());

        elevatorSim.update(0.020);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
        elevatorMech2d.setLength(elevatorSim.getPositionMeters());


        elevatorMech2d.setLength(elevatorSim.getPositionMeters());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));


    }
}
