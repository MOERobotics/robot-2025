package frc.robot.subsystem.simulations;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystem.SubMOErineClimber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.SimulationHelpers.*;

public class ClimberSim {
    private final double decelerationCoef = 1;
    private final double climberGearing = 4000;
    SparkMax climbMotor;
    SparkMaxSim climbMotorSim;
    CANcoderSimState climbEncoderSim;
    private final DCMotorSim climberSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 20, climberGearing), DCMotor.getNEO(1));
    public ClimberSim(SubMOErineClimber climber){
        this.climbMotor = climber.climbMotor;
        this.climbMotorSim = new SparkMaxSim(climbMotor, DCMotor.getNEO(1));
        this.climbEncoderSim = climber.climbEncoder.getSimState();
    }

    public void updateSimState(){
        climberSystem.setInputVoltage(climbMotor.getBusVoltage()*climbMotor.getAppliedOutput() * -1);
        climberSystem.setAngularVelocity(decelerate(climberSystem.getAngularVelocity(),decelerationCoef).in(RadiansPerSecond));
        climberSystem.update(0.02);

        climbMotorSim.iterate(getMotorVelocity(climberSystem.getAngularVelocity(),climberGearing).in(RPM),12.0,0.02);
        climbEncoderSim.setRawPosition(climberSystem.getAngularPosition());
        climbEncoderSim.setVelocity(climberSystem.getAngularVelocity());
    }
}
