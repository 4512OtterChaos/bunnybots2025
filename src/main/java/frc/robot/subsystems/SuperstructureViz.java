package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.ShooterArm.ShooterArm;
import frc.robot.subsystems.ShooterArm.ShooterArmConstants;
import frc.robot.subsystems.ShooterWheel.ShooterWheels;

public class SuperstructureViz {
    Intake intake;
    ShooterArm shooterArm;
    ShooterWheels shooterWheels;
    Funnel funnel; 

    private final Color8Bit kSetpointBaseColor = new Color8Bit(150, 0, 0);
    private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);

    private final double kSetpointWidth = 6;
    private final double kMechWidth = 9;

    private final double kDefaultFunnelDeg = 90;

    public SuperstructureViz (Intake intake, ShooterArm shooterArm, ShooterWheels shooterWheels, Funnel funnel) {
        this.intake = intake;
        this.shooterArm = shooterArm;
        this.shooterWheels = shooterWheels;
        this.funnel = funnel;
    } 

    Mechanism2d funnelMech = new Mechanism2d(.4, .4, new Color8Bit(0, 100, 150));
    MechanismRoot2d funnelMechRoot = funnelMech.getRoot("funnel", 0.3, 0.05);

    private final MechanismLigament2d mechBase = funnelMechRoot.append(
            new MechanismLigament2d("Base", FunnelConstants.kPivotHeight.in(Meters), 90, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechFunnel = mechBase.append(
            new MechanismLigament2d("Funnel", FunnelConstants.kFunnelLength.in(Meters), kDefaultFunnelDeg, kMechWidth, kMechBaseColor));
    
    private final MechanismLigament2d setpointBase = funnelMechRoot.append(
            new MechanismLigament2d("setpointBase", FunnelConstants.kPivotHeight.in(Meters), 90, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d setpointFunnel = setpointBase.append(
            new MechanismLigament2d("setpointFunnel", FunnelConstants.kFunnelLength.in(Meters), kDefaultFunnelDeg, kSetpointWidth,
                    kSetpointBaseColor));
    
    public void periodic() {
        mechFunnel.setAngle(90 - funnel.getAngle().in(Degrees));
        setpointFunnel.setAngle(90 - funnel.getTargetAngle().in(Degrees));
        
        SmartDashboard.putData("FunnelMech", funnelMech);
    }
}
