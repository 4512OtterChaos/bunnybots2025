package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.ShooterArm.ShooterArm;
import frc.robot.subsystems.ShooterArm.ShooterArmConstants;
import frc.robot.subsystems.ShooterWheel.ShooterWheelConstants;
import frc.robot.subsystems.ShooterWheel.ShooterWheels;
import frc.robot.util.MechanismCircle2d;

public class SuperstructureViz {
    Intake intake;
    ShooterArm shooterArm;
    ShooterWheels shooterWheels;
    Funnel funnel; 

    private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);
    private final Color8Bit kSetpointBaseColor = new Color8Bit(150, 0, 0);
    private final Color8Bit kWheelColor = new Color8Bit(0, 150, 0);

    private final double kSetpointWidth = 6;
    private final double kMechWidth = 9;

    private final double kDefaultFunnelDeg = 90;
    private final double kDefaultArmDeg = 88.186340;

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
    


    Mechanism2d shooterMech = new Mechanism2d(.5, .5, new Color8Bit(0, 100, 150));
    MechanismRoot2d shooterMechRoot = shooterMech.getRoot("ShooterArm", 0.25, 0.05);

    private final MechanismLigament2d mechBaseLeft = shooterMechRoot.append(
            new MechanismLigament2d("ArmBaseLeft", ShooterArmConstants.kBaseLength.in(Meters)/2, 180, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechArmLeft = mechBaseLeft.append(
            new MechanismLigament2d("ArmLeft", ShooterArmConstants.kPivotToWheels.in(Meters), -kDefaultArmDeg, kMechWidth, kMechBaseColor));
    private MechanismCircle2d leftShooterWheel = new MechanismCircle2d(mechArmLeft, 8, ShooterWheelConstants.kWheelDiameter, "roller", kWheelColor, true, true);
    private final MechanismLigament2d mechBaseRight = shooterMechRoot.append(
            new MechanismLigament2d("BaseRight", ShooterArmConstants.kBaseLength.in(Meters)/2, 0, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechArmRight = mechBaseRight.append(
            new MechanismLigament2d("ArmRight", ShooterArmConstants.kPivotToWheels.in(Meters), kDefaultArmDeg, kMechWidth, kMechBaseColor));
    private MechanismCircle2d rightShooterWheel = new MechanismCircle2d(mechArmRight, 8, ShooterWheelConstants.kWheelDiameter, "roller", kWheelColor, true, true);

    
    private final MechanismLigament2d setpointBaseLeft = shooterMechRoot.append(
            new MechanismLigament2d("setpointArmBaseLeft", ShooterArmConstants.kBaseLength.in(Meters)/2, 180, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d setpointArmLeft = setpointBaseLeft.append(
            new MechanismLigament2d("setpointArmLeft", ShooterArmConstants.kPivotToWheels.in(Meters), -kDefaultArmDeg, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d setpointBaseRight = shooterMechRoot.append(
            new MechanismLigament2d("setpointArmBaseRight", ShooterArmConstants.kBaseLength.in(Meters)/2, 0, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d setpointArmRight = setpointBaseRight.append(
            new MechanismLigament2d("setpointArmRight", ShooterArmConstants.kPivotToWheels.in(Meters), kDefaultArmDeg, kSetpointWidth, kSetpointBaseColor));
    

    
    
    public void periodic() {
        mechFunnel.setAngle(90 - funnel.getAngle().in(Degrees));
        setpointFunnel.setAngle(90 - funnel.getTargetAngle().in(Degrees));
        
        mechArmLeft.setAngle(180 + (kDefaultArmDeg - shooterArm.getAngle().in(Degrees)));
        mechArmRight.setAngle(180 - (kDefaultArmDeg - shooterArm.getAngle().in(Degrees)));

        leftShooterWheel.setAngle(shooterWheels.getAngle());
        rightShooterWheel.setAngle(shooterWheels.getAngle().times(-1));

        setpointArmLeft.setAngle(180 + (kDefaultArmDeg - ((shooterArm.getTargetVoltage().in(Volts) > 0) ? ShooterArmConstants.kInAngle.in(Degrees) : ShooterArmConstants.kOutAngle.in(Degrees))));
        setpointArmRight.setAngle(180 - (kDefaultArmDeg - ((shooterArm.getTargetVoltage().in(Volts) > 0) ? ShooterArmConstants.kInAngle.in(Degrees) : ShooterArmConstants.kOutAngle.in(Degrees))));
        
        SmartDashboard.putData("Mechs/FunnelMech", funnelMech);
        SmartDashboard.putData("Mechs/ShooterMech", shooterMech);
    }
}
