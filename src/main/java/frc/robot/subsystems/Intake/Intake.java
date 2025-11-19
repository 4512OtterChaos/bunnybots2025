package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.robot.subsystems.Intake.IntakeConstants.*;

public class Intake {
    TalonFX motor = new TalonFX(motorID);

    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    public Command setVoltageC(double voltage){
        return runOnce(()->setVoltage(voltage));
    }

    public Command setVoltageInC(){

    }
}

