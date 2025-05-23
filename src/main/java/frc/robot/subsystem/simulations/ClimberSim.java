package frc.robot.subsystem.simulations;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystem.SubMOErineClimber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.simulations.SimulationHelpers.*;

public class ClimberSim {
    private final double decelerationCoef = 1;
    private final double climberGearing = 4000;
    SparkMax climbMotor;
    SparkMaxSim climbMotorSim;
    SparkAbsoluteEncoderSim climbEncoderSim;
    private final DCMotorSim climberSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 5, climberGearing), DCMotor.getNEO(1));
    public ClimberSim(SubMOErineClimber climber){
        this.climbMotor = climber.climbMotor;
        this.climbMotorSim = new SparkMaxSim(climbMotor, DCMotor.getNEO(1));
        this.climbEncoderSim = new SparkAbsoluteEncoderSim(climbMotor);
    }

    public void updateSimState(){
        climberSystem.setInputVoltage(climbMotor.getBusVoltage()*climbMotor.getAppliedOutput());
        climberSystem.setAngularVelocity(decelerate(climberSystem.getAngularVelocity(),decelerationCoef).in(RadiansPerSecond));
        climberSystem.update(0.02);

        climbMotorSim.iterate(getMotorVelocity(climberSystem.getAngularVelocity(),climberGearing).in(RPM),12.0,0.02);
        climbEncoderSim.setPosition(climberSystem.getAngularPosition().in(Rotations));
        climbEncoderSim.setVelocity(climberSystem.getAngularVelocity().in(RPM));
    }
}
