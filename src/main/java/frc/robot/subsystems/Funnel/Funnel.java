package frc.robot.subsystems.Funnel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.subsystems.Funnel.FunnelConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Funnel extends SubsystemBase{
    TalonFX leftMotor = new TalonFX(kLeftMotorID);
    TalonFX rightMotor = new TalonFX(kRightMotorID);

    Angle targetAngle = kHomeAngle; 

    MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    public Funnel() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        leftMotor.setPosition(kHomeAngle);
    }

    @Override
    public void periodic() {
        
    }

    // public void setVoltage(double voltage) {
    //     leftMotor.setVoltage(voltage);
    // }

    public void setAngle(Angle angle) {
        angle = Degrees.of(MathUtil.clamp(angle.in(Degrees), kHomeAngle.in(Degrees), kMaxAngle.in(Degrees)));
        leftMotor.setControl(mmRequest.withPosition(angle)); //TODO: Set periodically?
        targetAngle = angle;
    }

    // public Command setVoltageC(double voltage) {
    //     return runOnce(()-> setVoltage(voltage));
    // }

    public Command setAngleC(Angle angle) {
        return runOnce(()-> setAngleC(angle));
    }


}
