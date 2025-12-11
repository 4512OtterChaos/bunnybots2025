package frc.robot.subsystems.ShooterArm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.util.OCUnits.PoundSquareInches;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ShooterArmConstants {
     
    public static final int kMotorID = 41;

    public static final double kVoltageIn = 0.5;
    public static final double kVoltageOut = -0.5;

    public static final int kGearRatio = 9; //9:1

    public static final MomentOfInertia kMomentOfInertia = PoundSquareInches.of(52.851002);

    public static final Angle kInAngle = Degrees.of(0);
    public static final Angle kOutAngle = Degrees.of(-2.862692); //physical maximum

    public static final Distance kBaseLength = Inches.of(6.720000);
    public static final Distance kPivotToWheels = Inches.of(7.538664);
    public static final Distance kArmLength = Inches.of(8.788664);

    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        feedback.SensorToMechanismRatio = kGearRatio;
        
        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kConfig.Slot0;
        control.kP = 5; 
        control.kI = 0;
        control.kD = 0;
        
        control.kS = 0.1;
        control.kV = 0;
        control.kA = 0;
    }
}