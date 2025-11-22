package frc.robot.subsystems.ShooterArm;

import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.subsystems.ShooterArm.ShooterArmConstants.*; 

public class ShooterArm {
    
    TalonFX motor = new TalonFX(motorID);
    
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
