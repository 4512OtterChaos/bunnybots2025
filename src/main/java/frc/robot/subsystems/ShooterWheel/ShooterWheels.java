package frc.robot.subsystems.ShooterWheel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.ShooterWheel.ShooterWheelConstants.*;

public class ShooterWheels extends SubsystemBase{
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();

    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

    private AngularVelocity targetVelocity = AngularVelocity.ofBaseUnits(0, RPM);

    public ShooterWheels() {
        leftMotor.getConfigurator().apply(kConfig);
        rightMotor.getConfigurator().apply(kConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    public Angle getAngle(){
        return leftMotor.getPosition().getValue();
    }

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    public void setVelocity(AngularVelocity velocity) {
        leftMotor.setControl(velocityRequest.withVelocity(velocity));
        targetVelocity = velocity;
    }

    public AngularVelocity getAngularVelocity(){
        return velocityStatus.getValue();
    }

    public boolean upToSpeed() {
        return Math.abs(targetVelocity.in(RPM) - getAngularVelocity().in(RPM)) < kVelocityTolerance.in(RPM);
    }

    public Command setVoltageC(double voltage) {
        return runOnce(()-> setVoltage(voltage)).withName("Set Voltage: " + voltage);
    }

    public Command setVelocityC(AngularVelocity velocity) {
        return run(()-> setVelocity(velocity)).until(upToSpeedT()).withName("Set Velocity: " + velocity);
    }

    public Trigger upToSpeedT() {
        return new Trigger(()-> upToSpeed());
    }


    
    //########## Simulation (IGNORE FOR NOW)

    final FlywheelSim motorSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2), //TODO: Replace with Minions
            kMomentOfInertia.in(KilogramSquareMeters),
            kGearRatio
        ),
        DCMotor.getKrakenX60(2) //TODO: Replace with Minions
    );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = leftMotor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;//TODO: Fix, idk what it means

        motorSimState.setSupplyVoltage(leftMotor.getSupplyVoltage().getValue());//TODO: Add friction? Also, idk that the voltage should be accessed like this

        motorSim.setInputVoltage(motorSimState.getMotorVoltage());

        motorSim.update(0.02);

        // motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * kGearRatio); TODO: Remove?
        motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60  * kGearRatio);
        //                                           shaft RPM --> rotations per second --> motor rotations per second
    }
}
