package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.ShooterArm.ShooterArm;
import frc.robot.subsystems.ShooterWheel.ShooterWheels;

public class Superstructure {
    Intake intake;
    ShooterArm shooterArm;
    ShooterWheels shooterWheels;
    Funnel funnel; 

    public Superstructure (Intake intake, ShooterArm shooterArm, ShooterWheels shooterWheels, Funnel funnel) {
        this.intake = intake;
        this.shooterArm = shooterArm;
        this.shooterWheels = shooterWheels;
        this.funnel = funnel;
    } 

    public Command shotSequence() {
        return sequence(
            parallel(
                funnel.setAngleC(Degrees.of(40)),
                shooterWheels.setVelocityC(RPM.of(430))
            ),
            shooterArm.setVoltageInC(),
            waitSeconds(0.5)
        ).withName("Shot Sequence");
    }

}
