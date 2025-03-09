package frc.robot.subsystem.simulations;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.CoralHead;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.simulations.SimulationHelpers.*;

public class CoralCollectorSim {
    private final double decelerationCoef = 10;
    private final SparkLimitSwitchSim coralLimitSim;
    private final SparkMax leftMotor, rightMotor;
    private final SparkMaxSim leftMotorSim, rightMotorSim;
    private final DCMotorSim leftMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.001, 1.0), DCMotor.getNeo550(1));
    private final DCMotorSim rightMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.001, 1.0), DCMotor.getNeo550(1));
    private final Timer timer = new Timer();

    public CoralCollectorSim(CoralHead coralCollector) {
        this.leftMotor = coralCollector.leftMotor;
        this.rightMotor = coralCollector.rightMotor;
        this.leftMotorSim = new SparkMaxSim(leftMotor, DCMotor.getNeo550(1));
        this.rightMotorSim = new SparkMaxSim(rightMotor, DCMotor.getNeo550(1));
        this.coralLimitSim = new SparkLimitSwitchSim(leftMotor, true);
        timer.start();
    }

    public void updateSimState() {
        leftMotorSystem.setInputVoltage(leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        leftMotorSystem.setState(leftMotorSystem.getAngularPositionRad(), decelerate(RPM.of(leftMotorSystem.getAngularVelocityRPM()), decelerationCoef).in(RadiansPerSecond));

        rightMotorSystem.setInputVoltage(rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        rightMotorSystem.setState(rightMotorSystem.getAngularPositionRad(), decelerate(RPM.of(rightMotorSystem.getAngularVelocityRPM()), decelerationCoef).in(RadiansPerSecond));

        leftMotorSystem.update(0.02);
        rightMotorSystem.update(0.02);

        leftMotorSim.iterate(leftMotorSystem.getAngularVelocityRPM(), 12.0, 0.02);
        rightMotorSim.iterate(rightMotorSystem.getAngularVelocityRPM(), 12.0, 0.02);

        if (leftMotorSim.getAppliedOutput() > 0 || rightMotorSim.getAppliedOutput() > 0.0) {
            timer.start();
        } else {
            timer.stop();
        }
        timer.advanceIfElapsed(6);

        coralLimitSim.setPressed(3 < timer.get() && timer.get() < 4);

        SmartDashboard.putNumber("Coral Timer", timer.get());
        SmartDashboard.putBoolean("Coral Limit", coralLimitSim.getPressed());

    }
}
