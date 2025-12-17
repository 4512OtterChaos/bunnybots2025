package frc.robot.subsystems.ShooterWheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.ShooterWheel.ShooterWheelConstants.*;

public class ShooterWheels extends SubsystemBase{
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private AngularVelocity targetVelocity = AngularVelocity.ofBaseUnits(0, RPM);

    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

    private final StatusSignal<Angle> positionStatus = leftMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();


    public ShooterWheels() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        SmartDashboard.putData("Shooter Wheels/Subsystem", this);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(positionStatus, velocityStatus, voltageStatus, statorStatus);
        log();
    }

    public Angle getAngle() {
        return positionStatus.getValue();
    }

    public AngularVelocity getVelocity(){
        return velocityStatus.getValue();
    }
    
    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    public Voltage getVoltage(){
        return voltageStatus.getValue();
    }

    public Current getCurrent(){
        return statorStatus.getValue();
    }

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    public void setVelocity(AngularVelocity velocity) {
        leftMotor.setControl(velocityRequest.withVelocity(velocity));
        targetVelocity = velocity;
    }

    public boolean upToSpeed() {
        return Math.abs(targetVelocity.in(RPM) - getVelocity().in(RPM)) < kVelocityTolerance.in(RPM);
    }

    public Command setVoltageC(double voltage) {
        return runOnce(()-> setVoltage(voltage)).withName("Set Voltage: " + voltage);
    }

    public Command setVelocityC(AngularVelocity velocity) {
        return run(()-> setVelocity(velocity)).until(upToSpeedT()).withName("Set Velocity: " + velocity);
    }

    public Trigger upToSpeedT() {
        return new Trigger(()-> upToSpeed()).debounce(kDebounceTime);
    }

    public void log(){
        SmartDashboard.putNumber("Shooter Wheels/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Shooter Wheels/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Shooter Wheels/Target RPM", getTargetVelocity().in(RPM));
        SmartDashboard.putNumber("Shooter Wheels/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter Wheels/Current", getCurrent().in(Amps));
    }
}
