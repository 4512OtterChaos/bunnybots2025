// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureViz;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.ShooterArm.ShooterArm;
import frc.robot.subsystems.ShooterWheel.ShooterWheels;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class RobotContainer {
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Intake intake = new Intake();
  public final ShooterArm shooterArm = new ShooterArm();
  public final ShooterWheels shooterWheels = new ShooterWheels();
  public final Funnel funnel = new Funnel();
  public final Superstructure superstructure = new Superstructure(intake, shooterArm, shooterWheels, funnel);
  public final SuperstructureViz superstructureViz = new SuperstructureViz(intake, shooterArm, shooterWheels, funnel);
  
  public final CommandXboxController controller = new CommandXboxController(0);

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  
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

    controller.povRight().whileTrue(shooterArm.setVoltageInC().repeatedly());

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
        drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    // reset the field-centric heading on left bumper press
    controller.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
    
  }

  public void periodic() {
    superstructureViz.periodic();
  }
  
  public void teleopPeriodic() {
    
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
