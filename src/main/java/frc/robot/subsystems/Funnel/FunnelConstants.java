package frc.robot.subsystems.Funnel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class FunnelConstants {
    public static final int kLeftMotorID = 51;
    public static final int kRightMotorID = 52;

    public static final Angle kHomeAngle = Degrees.of(0);
    public static final Angle kMaxAngle = Degrees.of(60);

    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kConfig.Slot0;
        control.kP = 0.5; 
        control.kI = 0;
        control.kD = 0;
        
        control.GravityType = GravityTypeValue.Arm_Cosine;
        control.kG = 0.25;
        control.kS = 0.1;
        control.kV = 0;
        control.kA = 0;

        MotionMagicConfigs mm = kConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = RotationsPerSecond.of(2).in(RotationsPerSecond);
        mm.MotionMagicAcceleration = RotationsPerSecondPerSecond.of(5).in(RotationsPerSecondPerSecond);
    }
}
