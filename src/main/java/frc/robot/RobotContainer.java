// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer {
  Intake intake = new Intake();
  
  CommandXboxController controller = new CommandXboxController(0);
  
  public RobotContainer() {
    configureBindings();
  }
  
  private void configureBindings() {
    controller.rightTrigger().whileTrue(intake.setVoltageInC().repeatedly().finallyDo(()->intake.setVoltage(0)));
    controller.leftTrigger().whileTrue(intake.setVoltageOutC().repeatedly().finallyDo(()->intake.setVoltage(0)));
  }
  
  public void teleopPeriodic(){
    
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
