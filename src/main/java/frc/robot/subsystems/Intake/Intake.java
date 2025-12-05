package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.robot.subsystems.Intake.IntakeConstants.*;

public class Intake extends SubsystemBase{
    TalonFX motor = new TalonFX(kMotorID);

    public Intake() {
        motor.getConfigurator().apply(kConfig);
    }

    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    public Command setVoltageC(double voltage){
        return runOnce(()->setVoltage(voltage));
    }

    public Command setVoltageInC(){
        return setVoltageC(kVoltageIn);
    }

    public Command setVoltageOutC() {
        return setVoltageC(kVoltageOut);
    }
}



