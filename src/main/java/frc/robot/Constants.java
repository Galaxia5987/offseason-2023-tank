package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Constants {
    public static final boolean ENABLE_VOLT_COMP = true;
    public static final int CONFIG_VOLT_COMP_SATURATION = 10; //[V]
    public static final TalonFXInvertType CLOCKWISE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType COUNTER_CLOCKWISE = TalonFXInvertType.CounterClockwise;
    public static final double TICKS_PER_M = 2048 / (2 * Math.PI * 0.0666); //[ticks/meters]
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final int TALON_TIME_OUT = 10; //[ms]
    public static final double DEAD_BEND = 0.05; //[meters]

}
