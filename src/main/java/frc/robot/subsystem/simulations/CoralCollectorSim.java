package frc.robot.subsystem.simulations;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystem.CoralHead;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.simulations.SimulationHelpers.*;

public class CoralCollectorSim {
    private final double decelerationCoef = 10;
    private final SparkLimitSwitchSim coralLimitSimFront, coralLimitSimBack;
    private final SparkMax leftMotor, rightMotor;
    private final SparkMaxSim leftMotorSim, rightMotorSim;
    private final DCMotorSim leftMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.001, 1.0), DCMotor.getNeo550(1));
    private final DCMotorSim rightMotorSystem = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.001, 1.0), DCMotor.getNeo550(1));
    private final Timer timer = new Timer();
    private double collectionProgress;

    public CoralCollectorSim(CoralHead coralCollector) {
        this.leftMotor = coralCollector.leftMotor;
        this.rightMotor = coralCollector.rightMotor;
        this.leftMotorSim = new SparkMaxSim(leftMotor, DCMotor.getNeo550(1));
        this.rightMotorSim = new SparkMaxSim(rightMotor, DCMotor.getNeo550(1));
        this.coralLimitSimBack = new SparkLimitSwitchSim(leftMotor, true);
        this.coralLimitSimFront = new SparkLimitSwitchSim(leftMotor, false);
        collectionProgress = 5.0;
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
            collectionProgress += rightMotorSim.getAppliedOutput();
        }
        if(collectionProgress > 9){
            timer.start();
            if (timer.hasElapsed(1)){
                collectionProgress = 0;
                timer.stop();
                timer.reset();
            }
        }

//        coralLimitSimBack.setPressed(1 < timer.get() && timer.get() < 3);
        coralLimitSimBack.setPressed(1 < collectionProgress && collectionProgress < 6);
        coralLimitSimFront.setPressed(4 < collectionProgress && collectionProgress < 9);
//        coralLimitSimFront.setPressed(2 < timer.get() && timer.get() < 4);
        Logger.recordOutput("Coral Progress", collectionProgress);

    }
}
