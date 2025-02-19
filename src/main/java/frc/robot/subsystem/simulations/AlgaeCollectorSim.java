package frc.robot.subsystem.simulations;


import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.AlgaeCollector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.SimulationHelpers.*;

public class AlgaeCollectorSim {
    private final double armDecelerationCoef = 50;
    private final double wheelDecelerationCoef = 20;
    private final double armGearing = 9.0;
    private final double wheelGearing = 9.0;
    private final SendableChooser<Boolean> algaeCollection = new SendableChooser<>();
    private final SparkMax algaeWheel, algaeArm;
    private final SparkMaxSim algaeWheelSim, algaeArmSim;
    private final SparkLimitSwitchSim algaeWheelLimitSim;
    private final SparkAbsoluteEncoderSim algaeArmEncoderSim;
    private final SingleJointedArmSim algaeArmSystem = new SingleJointedArmSim(DCMotor.getNEO(1), armGearing, 0.1, Units.inchesToMeters(16), 0, Math.PI/2, true, Math.PI/2);
    private final DCMotorSim algaeWheelSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.01, wheelGearing), DCMotor.getNEO(1));

    public AlgaeCollectorSim(AlgaeCollector algaeCollector) {
        this.algaeWheel = algaeCollector.algaeWheel;
        this.algaeArm = algaeCollector.algaeArm;
        this.algaeWheelSim = new SparkMaxSim(algaeWheel, DCMotor.getNEO(1));
        this.algaeArmSim = new SparkMaxSim(algaeArm, DCMotor.getNEO(1));
        this.algaeWheelLimitSim = new SparkLimitSwitchSim(algaeWheel,false);
        this.algaeArmEncoderSim = new SparkAbsoluteEncoderSim(algaeArm);
        algaeCollection.setDefaultOption("Algae Collection Successful",true);
        algaeCollection.addOption("Algae Collection Unsuccessful",false);
        SmartDashboard.putData(algaeCollection);
    }

    public void updateSimState(){
        algaeArmSystem.setInputVoltage(algaeArm.getBusVoltage()*algaeArm.getAppliedOutput());
        algaeArmSystem.setState(algaeArmSystem.getAngleRads(),decelerate(RadiansPerSecond.of(algaeArmSystem.getVelocityRadPerSec()),armDecelerationCoef).in(RadiansPerSecond));

        algaeWheelSystem.setInputVoltage(algaeWheel.getBusVoltage()*algaeWheel.getAppliedOutput());
        algaeWheelSystem.setState(algaeWheelSystem.getAngularPositionRad(),decelerate(RadiansPerSecond.of(algaeWheelSystem.getAngularVelocityRadPerSec()),wheelDecelerationCoef).in(RadiansPerSecond));

        algaeArmSystem.update(0.02);
        algaeWheelSystem.update(0.02);

        algaeArmSim.iterate(getMotorVelocity(RadiansPerSecond.of(algaeArmSystem.getVelocityRadPerSec()),armGearing).in(RPM),12.0,0.02);
        algaeWheelSim.iterate(getMotorVelocity(algaeWheelSystem.getAngularVelocity(),wheelGearing).in(RPM),12.0,0.02);

        algaeWheelLimitSim.setPressed(algaeArmSystem.getAngleRads()<Units.degreesToRadians(40+Math.random()*5)&&algaeCollection.getSelected());

        algaeArmEncoderSim.setVelocity(RadiansPerSecond.of(algaeArmSystem.getVelocityRadPerSec()).in(RPM));
        algaeArmEncoderSim.setPosition(Radians.of(algaeArmSystem.getAngleRads()).in(Rotations));
        SmartDashboard.putNumber("Arm Angle",algaeArmSystem.getAngleRads());
    }

}
