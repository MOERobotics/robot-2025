package frc.robot.subsystem.simulations;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystem.SubMOErineElevator;
import frc.robot.subsystem.interfaces.ElevatorControl;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.simulations.SimulationHelpers.*;

public class RobotElevatorSim {//TODO: Fix constants
    private final double decelerationCoefPivot = 30;
    private final double decelerationCoefHeight = 50;
    private final double pivotGearing = 2000.0;
    private final double heightGearing = 49;
    private final double drumRadius = 0.0221488;
    private final double carriageMassKg = 1;
    private final SparkMaxSim elevatorHeightSim, elevatorPivotSim;
    private final SparkLimitSwitchSim elevatorLimitSim;
    private final SparkMax elevatorHeightMotor, elevatorPivotMotor;
    private final SparkAbsoluteEncoderSim tiltEncoderSim;
    private final AnalogInputSim extensionSensorSim;
    private final DCMotorSim elevatorPivotSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 20, pivotGearing), DCMotor.getNeo550(1));
    private final ElevatorSim elevatorHeightSystem = new ElevatorSim(DCMotor.getNEO(1), heightGearing, carriageMassKg, drumRadius, 0.44, 1.81, true, 0.5);

    public RobotElevatorSim(SubMOErineElevator elevator) {
        this.elevatorHeightMotor = elevator.elevatorExtensionMotor;
        this.elevatorPivotMotor = elevator.elevatorPivotMotor;
        this.elevatorHeightSim = new SparkMaxSim(elevatorHeightMotor, DCMotor.getNEO(1));
        this.elevatorPivotSim = new SparkMaxSim(elevatorPivotMotor, DCMotor.getNEO(1));
        this.elevatorLimitSim = new SparkLimitSwitchSim(elevatorHeightMotor,false);
        this.tiltEncoderSim = new SparkAbsoluteEncoderSim(elevatorPivotMotor);
        this.extensionSensorSim = new AnalogInputSim(elevator.extensionSensor);
    }

    public void updateSimState() {
        elevatorHeightSystem.setInputVoltage(elevatorHeightMotor.getBusVoltage() * elevatorHeightMotor.getAppliedOutput());
        elevatorHeightSystem.setState(elevatorHeightSystem.getPositionMeters(), getElevatorHeightVelocity(decelerate(getHeightMotorVelocity(), decelerationCoefHeight)));

        elevatorPivotSystem.setInputVoltage(elevatorPivotMotor.getBusVoltage() * elevatorPivotMotor.getAppliedOutput());
        elevatorPivotSystem.setAngularVelocity(decelerate(elevatorPivotSystem.getAngularVelocity(), decelerationCoefPivot).in(RadiansPerSecond));
        if(elevatorPivotSystem.getAngularPosition().lt(Degrees.of(-45))){
            elevatorPivotSystem.setState(-Math.PI/4,0);
        }
        if(elevatorPivotSystem.getAngularPosition().gt(Degrees.of(45))){
            elevatorPivotSystem.setState(Math.PI/4,0);
        }

        elevatorHeightSystem.update(0.02);
        elevatorPivotSystem.update(0.02);

        elevatorLimitSim.setPressed(elevatorHeightSystem.getPositionMeters() > ElevatorControl.ElevatorHeight.STOW.measure.in(Meters)-0.08);

        elevatorHeightSim.iterate(getHeightMotorVelocity().in(RPM), 12, 0.02);
        elevatorPivotSim.iterate(elevatorPivotSystem.getAngularVelocityRPM() * pivotGearing, 12, 0.02);
        tiltEncoderSim.setPosition(elevatorPivotSystem.getAngularPosition().in(Rotations));
        tiltEncoderSim.setVelocity(elevatorPivotSystem.getAngularVelocity().in(RPM));
        extensionSensorSim.setVoltage((elevatorHeightSystem.getPositionMeters()*100-21.80649)/35.17649);
    }

    public AngularVelocity getHeightMotorVelocity() {
        return RevolutionsPerSecond.of(elevatorHeightSystem.getVelocityMetersPerSecond() * heightGearing / (2 * Math.PI * drumRadius));
    }

    public double getElevatorHeightVelocity(AngularVelocity heightMotorVelocity) {
        return heightMotorVelocity.in(RevolutionsPerSecond) / heightGearing * (2 * Math.PI * drumRadius);
    }

}
