package frc.robot.subsystems.ShooterArm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.robot.subsystems.ShooterArm.ShooterArmConstants.*; 

public class ShooterArm {
    
    TalonFX leftMotor = new TalonFX(kMotorID);
    TalonFX rightMotor = new TalonFX(kMotorID);

    public ShooterArm() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }
    
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }
}
