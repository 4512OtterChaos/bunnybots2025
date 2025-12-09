// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.ShooterArm.ShooterArm;
import frc.robot.subsystems.ShooterWheel.ShooterWheels;

public class RobotContainer {
  Intake intake = new Intake();
  ShooterArm shooterArm = new ShooterArm();
  ShooterWheels shooterWheels = new ShooterWheels();
  Funnel funnel = new Funnel();
  Superstructure superstructure = new Superstructure(intake, shooterArm, shooterWheels, funnel);
  
  CommandXboxController controller = new CommandXboxController(0);
  
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    intake.setDefaultCommand(intake.setVoltageC(0));
    shooterArm.setDefaultCommand(shooterArm.setVoltageOutC());
    shooterWheels.setDefaultCommand(shooterWheels.setVoltageC(0));
  }
  
  private void configureBindings() {
    controller.rightTrigger().whileTrue(intake.setVoltageInC().repeatedly());
    controller.leftTrigger().whileTrue(intake.setVoltageOutC().repeatedly());

    controller.rightBumper().whileTrue(superstructure.shotSequence());

    controller.povUp().onTrue(funnel.setAngleC(Degrees.of(50)));
    controller.povDown().onTrue(funnel.setAngleC(Degrees.of(0)));
  }
  
  public void teleopPeriodic(){
    
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
