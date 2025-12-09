package frc.robot.subsystems.Funnel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.security.PublicKey;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import static frc.robot.util.OCUnits.*;

public class FunnelConstants {
    public static final int kLeftMotorID = 51;
    public static final int kRightMotorID = 52;

    public static final int kGearRatio = 9; //9:1

    public static final MomentOfInertia kMomentOfInertia = PoundSquareInches.of(425.845093);

    public static final Angle kHomeAngle = Degrees.of(0);
    public static final Angle kMaxAngle = Degrees.of(50); //53.97 degrees is the physical maximum (not touching but pretty close)

    public static final Distance kPivotHeight = Inches.of(3.72);
    public static final Distance kFunnelLength = Inches.of(10.04);

    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;//TODO: Teach
        feedback.SensorToMechanismRatio = kGearRatio;//TODO: Teach

        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kConfig.Slot0;
        control.kP = 6; 
        control.kI = 0;
        control.kD = 0;
        
        control.GravityType = GravityTypeValue.Arm_Cosine;
        control.kG = 0.03;
        control.kS = 0.1;
        control.kV = 0;
        control.kA = 0;

        MotionMagicConfigs mm = kConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = RotationsPerSecond.of(2).in(RotationsPerSecond);
        mm.MotionMagicAcceleration = RotationsPerSecondPerSecond.of(5).in(RotationsPerSecondPerSecond);
    }
}
