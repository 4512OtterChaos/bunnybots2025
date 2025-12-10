package frc.robot.subsystems.ShooterArm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.robot.subsystems.ShooterArm.ShooterArmConstants.*; 

public class ShooterArm extends SubsystemBase{
    
    private TalonFX leftMotor = new TalonFX(kMotorID);
    private TalonFX rightMotor = new TalonFX(kMotorID);

    private double targetVoltage = kVoltageIn;

    public ShooterArm() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        leftMotor.setPosition(kInAngle);
    }

    @Override
    public void periodic() {
        log();
    }

    public Angle getAngle() {
        return leftMotor.getPosition().getValue();
    }

    public double getTargetVoltage() {
        return targetVoltage;
    }
    
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        targetVoltage = voltage;
    }

    public Command setVoltageC(double voltage){
        return runOnce(()->setVoltage(voltage)).withName("Set Voltage: " + voltage);
    }

    public Command setVoltageInC(){
        return setVoltageC(kVoltageIn).withName("Set Voltage In");
    }

    public Command setVoltageOutC() {
        return setVoltageC(kVoltageOut).withName("Set Voltage Out");
    }


    
    //########## Simulation (IGNORE FOR NOW)

    public void log() {
        SmartDashboard.putNumber("ShooterArm/ShooterArm Native", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ShooterArm/ShooterArm Degrees", leftMotor.getPosition().getValue().in(Degrees));
        SmartDashboard.putNumber("ShooterArm/Motor Current", leftMotor.getStatorCurrent().getValue().in(Amps));
        // SmartDashboard.putBoolean("Funnel/isStalled", isStalled());
        SmartDashboard.putNumber("ShooterArm/Motor Voltage", leftMotor.getMotorVoltage().getValue().in(Volts));
        // SmartDashboard.putNumber("ShooterArm/Target Motor Voltage", targetVoltage);
        // SmartDashboard.putNumber("Funnel/Motor Target Voltage", targetVoltage);
        SmartDashboard.putNumber("ShooterArm/Motor Velocity", leftMotor.getVelocity().getValue().in(RPM));
        // SmartDashboard.putBoolean("Funnel/Motor Stalled", isStalled());
        SmartDashboard.putNumber("ShooterArm/Time", Timer.getFPGATimestamp());
    }
    
    SingleJointedArmSim shooterArmSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX60(1),
            kMomentOfInertia.in(KilogramSquareMeters),
            kGearRatio
        ),
        DCMotor.getKrakenX60(1),
        kGearRatio,
        kArmLength.in(Meters),
        kOutAngle.in(Radians),
        kInAngle.in(Radians),
        false,//TODO:Include gravity?
        kInAngle.in(Radians));


    DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            kMomentOfInertia.in(KilogramSquareMeters),
            kGearRatio
        ),
        DCMotor.getKrakenX60(1)
    );

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
        shooterArmSim.setInput(voltage);
		shooterArmSim.update(0.02);
    }
}
