package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleSim{
    private final double pivotReduction = 46;
    private final double decelerationCoef = 30;
    private final SparkMaxSim driveMotorSim, pivotMotorSim;
    private final SparkMax driveMotor, pivotMotor;
    private final CANcoderSimState pivotEncoderSim;
    private final DCMotorSim driveMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.005, 1.0), DCMotor.getNEO(1));
    private final DCMotorSim pivotMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.01, 1.0), DCMotor.getNEO(1));
    Angle heading;

    public SwerveModuleSim(Angle heading, SparkMax driveMotor, SparkMax pivotMotor, CANcoder pivotEncoder) {
        this.heading = heading;
        this.driveMotor = driveMotor;
        this.pivotMotor = pivotMotor;
        this.driveMotorSim = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));
        this.pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
        this.pivotEncoderSim = pivotEncoder.getSimState();
    }

    public void updateSimState() {
        driveMotorSystem.setInput(driveMotor.get());
//        driveMotorSystem.setAngularVelocity(decelerate(driveMotorSystem.getAngularVelocity()).in(RadiansPerSecond));
        pivotMotorSystem.setAngularVelocity(decelerate(pivotMotorSystem.getAngularVelocity()).in(RadiansPerSecond));
        pivotMotorSystem.setInput(pivotMotor.get());
        driveMotorSystem.update(.02);
        pivotMotorSystem.update(.02);
        driveMotorSim.iterate(driveMotorSystem.getAngularVelocityRPM(),12.0,0.02);
        pivotMotorSim.iterate(pivotMotorSystem.getAngularVelocityRPM(),12.0,0.02);
        pivotMotorSim.getAlternateEncoderSim().iterate(pivotMotorSystem.getAngularVelocityRPM(),.02);
        pivotEncoderSim.setRawPosition(pivotMotorSystem.getAngularPosition().div(pivotReduction).unaryMinus().plus(heading));
        pivotEncoderSim.setVelocity(pivotMotorSystem.getAngularVelocity().div(pivotReduction).unaryMinus());
    }

    public AngularVelocity decelerate(AngularVelocity velocity){
        if(velocity.isNear(RPM.zero(),0.01)){
            return RPM.zero();
        }else {
            return velocity.abs(RPM)>decelerationCoef?velocity.minus(RPM.of(Math.copySign(decelerationCoef,velocity.in(RPM)))):RPM.zero();
        }
    }

}