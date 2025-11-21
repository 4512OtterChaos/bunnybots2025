package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

public class Shooter {
    TalonFX leftMotor = new TalonFX(leftMotorID);
    TalonFX rightMotor = new TalonFX(rightMotorID);
}
