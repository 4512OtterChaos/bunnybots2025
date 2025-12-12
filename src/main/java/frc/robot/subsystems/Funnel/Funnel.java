package frc.robot.subsystems.Funnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Funnel.FunnelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Funnel extends SubsystemBase{
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private Angle targetAngle = kHomeAngle; 

    private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    private final StatusSignal<Angle> positionStatus = leftMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();


    public Funnel() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        SmartDashboard.putData("Funnel/Subsystem", this);

        leftMotor.setPosition(kHomeAngle);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(positionStatus, velocityStatus, voltageStatus, statorStatus);

        leftMotor.setControl(mmRequest.withPosition(targetAngle));

        log();
    }

    public Angle getAngle() {
        return positionStatus.getValue();
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public AngularVelocity getVelocity() {
        return velocityStatus.getValue();
    }

    public Voltage getVoltage() {
        return voltageStatus.getValue();
    }

    public Current getCurrent() {
        return statorStatus.getValue();
    }

    // public void setVoltage(double voltage) {
    //     leftMotor.setVoltage(voltage);
    // }

    public void setAngle(Angle angle) {
        angle = Degrees.of(MathUtil.clamp(angle.in(Degrees), kHomeAngle.in(Degrees), kMaxAngle.in(Degrees)));
        targetAngle = angle;
    }

    // public Command setVoltageC(double voltage) {
    //     return runOnce(()-> setVoltage(voltage));
    // }

    public Command setAngleC(Angle angle) {
        return runOnce(()-> setAngle(angle)).withName("Set Degrees: " + angle.in(Degrees));
    }

    public void log() {
        SmartDashboard.putNumber("Funnel/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Funnel/Target Angle Degrees", getTargetAngle().in(Degrees));
        SmartDashboard.putNumber("Funnel/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Funnel/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Funnel/Current", getCurrent().in(Amps));
    }


    
    //########## Simulation (IGNORE FOR NOW)
    
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
