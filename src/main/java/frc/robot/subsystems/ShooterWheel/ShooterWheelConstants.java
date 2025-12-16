package frc.robot.subsystems.ShooterWheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class ShooterWheelConstants {
    public static final int kLeftMotorID = 31;
    public static final int kRightMotorID = 32;

    public static final AngularVelocity kVelocityTolerance = RPM.of(30);
    public static final double kDebounceTime = 0.2;

    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kConfig.Slot0;
        control.kP = 0.1; 
        control.kI = 0;
        control.kD = 0;
        
        control.kS = 0.1;
        control.kV = 0;
        control.kA = 0;
    }

}