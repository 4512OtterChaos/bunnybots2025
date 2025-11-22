package frc.robot.subsystems.ShooterWheel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.AngularVelocity;

import static frc.robot.subsystems.ShooterWheel.ShooterWheelConstants.*;

public class ShooterWheels {
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
    }

    public AngularVelocity getAngularVelocity(){
        return velocityStatus.getValue();
    }

    public boolean upToSpeed() {
        return Math.abs(targetVelocity.in(RPM) - getAngularVelocity().in(RPM)) < kVelocityTolerance.in(RPM);
    }
}
