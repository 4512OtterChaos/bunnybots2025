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

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }
}
