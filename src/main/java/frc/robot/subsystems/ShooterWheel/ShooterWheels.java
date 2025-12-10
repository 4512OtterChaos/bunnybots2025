package frc.robot.subsystems.ShooterWheel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.ShooterWheel.ShooterWheelConstants.*;

public class ShooterWheels extends SubsystemBase{
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();

    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

    private AngularVelocity targetVelocity = AngularVelocity.ofBaseUnits(0, RPM);

    public ShooterWheels() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    public void setVelocity(AngularVelocity velocity) {
        leftMotor.setControl(velocityRequest.withVelocity(velocity));
        targetVelocity = velocity;
    }

    public AngularVelocity getAngularVelocity(){
        return velocityStatus.getValue();
    }

    public boolean upToSpeed() {
        return Math.abs(targetVelocity.in(RPM) - getAngularVelocity().in(RPM)) < kVelocityTolerance.in(RPM);
    }

    public Command setVoltageC(double voltage) {
        return runOnce(()-> setVoltage(voltage)).withName("Set Voltage: " + voltage);
    }

    public Command setVelocityC(AngularVelocity velocity) {
        return runOnce(()-> setVelocity(velocity)).withName("Set Velocity: " + velocity);
    }

    public Trigger upToSpeedT() {
        return new Trigger(()-> upToSpeed());
    }
}
