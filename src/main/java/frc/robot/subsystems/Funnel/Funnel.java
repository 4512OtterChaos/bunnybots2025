package frc.robot.subsystems.Funnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Funnel.FunnelConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Funnel extends SubsystemBase{
    TalonFX leftMotor = new TalonFX(kLeftMotorID);
    TalonFX rightMotor = new TalonFX(kRightMotorID);

    Angle targetAngle = kHomeAngle; 

    MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    public Funnel() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        leftMotor.setPosition(kHomeAngle);
    }

    @Override
    public void periodic() {
        // ### Simulation

        visualizeState(leftMotor.getPosition().getValue());
        visualizeSetpoint(targetAngle);

        log();
    }

    // public void setVoltage(double voltage) {
    //     leftMotor.setVoltage(voltage);
    // }

    public void setAngle(Angle angle) {
        angle = Degrees.of(MathUtil.clamp(angle.in(Degrees), kHomeAngle.in(Degrees), kMaxAngle.in(Degrees)));
        leftMotor.setControl(mmRequest.withPosition(angle)); //TODO: Set periodically?
        targetAngle = angle;
    }

    // public Command setVoltageC(double voltage) {
    //     return runOnce(()-> setVoltage(voltage));
    // }

    public Command setAngleC(Angle angle) {
        return runOnce(()-> setAngleC(angle));
    }


    
    //########## Simulation (IGNORE FOR NOW)

    public void log() {
        SmartDashboard.putNumber("Funnel/Funnel Native", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Funnel/Funnel Degrees", leftMotor.getPosition().getValue().in(Degrees));
        SmartDashboard.putNumber("Funnel/Funnel Target Degrees", targetAngle.in(Degrees));
        SmartDashboard.putNumber("Funnel/Motor Current", leftMotor.getStatorCurrent().getValue().in(Amps));
        // SmartDashboard.putBoolean("Funnel/isStalled", isStalled());
        SmartDashboard.putNumber("Funnel/Motor Voltage", leftMotor.getMotorVoltage().getValue().in(Volts));
        // SmartDashboard.putNumber("Funnel/Motor Target Voltage", targetVoltage);
        SmartDashboard.putNumber("Funnel/Motor Velocity", leftMotor.getVelocity().getValue().in(RPM));
        // SmartDashboard.putBoolean("Funnel/Motor Stalled", isStalled());
        SmartDashboard.putNumber("Funnel/Time", Timer.getFPGATimestamp());
        SmartDashboard.putData("Funnel/Mech2d", mech);
    }
    
    Mechanism2d mech = new Mechanism2d(.8, .8, new Color8Bit(0, 100, 150));
    MechanismRoot2d mechRoot = mech.getRoot("funnel", 0.4, 0.1);

    private final Color8Bit kSetpointBaseColor = new Color8Bit(150, 0, 0);
    private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);

    private final double kSetpointWidth = 6;
    private final double kMechWidth = 9;

    private final double kDefaultFunnelDeg = 90;

    private final MechanismLigament2d mechStart = mechRoot.append(
            new MechanismLigament2d("Arm", kPivotHeight.in(Meters), 90, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechFunnel = mechStart.append(
            new MechanismLigament2d("Intake", kFunnelLength.in(Meters), kDefaultFunnelDeg, kMechWidth, kMechBaseColor));
    
    private final MechanismLigament2d setpointStart = mechRoot.append(
            new MechanismLigament2d("setpointArmBase", kPivotHeight.in(Meters), 90, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d setpointIntake = setpointStart.append(
            new MechanismLigament2d("setpointArm", kFunnelLength.in(Meters), kDefaultFunnelDeg, kSetpointWidth,
                    kSetpointBaseColor));
    
    SingleJointedArmSim funnelSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX60(2),
            kMomentOfInertia.in(KilogramSquareMeters),
            kGearRatio
        ),
        DCMotor.getKrakenX60(2),
        kGearRatio,
        kFunnelLength.in(Meters),
        kHomeAngle.in(Radians),
        kMaxAngle.in(Radians),
        true,
        kHomeAngle.in(Radians));


    DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2),
            kMomentOfInertia.in(KilogramSquareMeters),
            kGearRatio
        ),
        DCMotor.getKrakenX60(2)
    );
    
    public void visualizeState(Angle funnelAngle) {
        mechFunnel.setAngle(90 - funnelAngle.in(Degrees));
    }
    
    public void visualizeSetpoint(Angle targetAngle) {
        setpointIntake.setAngle(90 - targetAngle.in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = leftMotor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;//TODO: Fix, idk what it means

        motorSimState.setSupplyVoltage(leftMotor.getSupplyVoltage().getValue());//TODO: Add friction? Also, idk that the voltage should be accessed like this

        motorSim.setInputVoltage(motorSimState.getMotorVoltage());

        motorSim.update(0.02);

        motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * kGearRatio);
        motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60  * kGearRatio);
        //                                           shaft RPM --> rotations per second --> motor rotations per second
        double voltage = motorSim.getInputVoltage();
        funnelSim.setInput(voltage);
		funnelSim.update(0.02);
    }
}
