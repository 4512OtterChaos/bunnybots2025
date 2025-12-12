package frc.robot.subsystems.ShooterArm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import static frc.robot.subsystems.ShooterArm.ShooterArmConstants.*; 

public class ShooterArm extends SubsystemBase{
    
    private TalonFX leftMotor = new TalonFX(kMotorID);
    private TalonFX rightMotor = new TalonFX(kMotorID);

    private Voltage targetVoltage = Volts.of(0);

    private final StatusSignal<Angle> positionStatus = leftMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();


    public ShooterArm() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
        SmartDashboard.putData("Shooter Arm/Subsystem", this);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(positionStatus, velocityStatus, voltageStatus, statorStatus);
        log();
    }

    public Angle getAngle() {
        return positionStatus.getValue();
    }

    public AngularVelocity getVelocity() {
        return velocityStatus.getValue();
    }

    public Voltage getVoltage() {
        return voltageStatus.getValue();
    }

    public Voltage getTargetVoltage() {
        return targetVoltage;
    }

    public Current getCurrent() {
        return statorStatus.getValue();
    }
    
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        targetVoltage = Volts.of(voltage);
    }

    public Command setVoltageC(double voltage){
        return runOnce(()->setVoltage(voltage)).withName("Set Voltage: " + voltage);
    }

    public Command setVoltageInC(){
        return setVoltageC(kVoltageIn).withName("Set Voltage In");//TODO: Adjust to be smaller when hits hardstop (stall detection)
    }

    public Command setVoltageOutC() {
        return setVoltageC(kVoltageOut).withName("Set Voltage Out");
    }

    public void log() {
        SmartDashboard.putNumber("Shooter Arm/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Shooter Arm/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Shooter Arm/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter Arm/Target Voltage", getTargetVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter Arm/Current", getCurrent().in(Amps));
    }


    
    //########## Simulation (IGNORE FOR NOW)
    
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
