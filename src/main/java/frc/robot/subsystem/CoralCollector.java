package frc.robot.subsystem;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.MOESubsystem;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.RPM;

public class CoralCollector extends MOESubsystem<CoralCollectorInputsAutoLogged> implements CoralCollectorControl {

    public SparkMax leftMotor ;
    public SparkMax rightMotor;



    public CoralCollector(SparkMax leftMotor, SparkMax rightMotor){
        this.setSensors(new CoralCollectorInputsAutoLogged());
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        getSensors().hasCoral = false;
        getSensors().inFrontReef = false;
        getSensors().velocityRight = RPM.zero();
        getSensors().velocityLeft = RPM.zero();
    }

    @Override
    public boolean hasCoral() {
        return leftMotor.getReverseLimitSwitch().isPressed();
    }


    @Override
    public boolean inFrontReef() {
        return false;
    }


    @Override
    public void readSensors(CoralCollectorInputsAutoLogged sensors) {
        sensors.hasCoral = leftMotor.getReverseLimitSwitch().isPressed();
        sensors.inFrontReef = false;
        sensors.velocityLeft = RPM.of(leftMotor.getEncoder().getVelocity());
        sensors.velocityRight = RPM.of(rightMotor.getEncoder().getVelocity());
    }


    @Override
    public void setCoralVelocity(AngularVelocity leftPower, AngularVelocity rightPower) {
        leftMotor.set(leftPower.in(RPM));
        rightMotor.set(rightPower.in(RPM));
    }

    @Override
    public AngularVelocity getLeftVelocity() {
        return RPM.of(leftMotor.get());
    }

    @Override
    public AngularVelocity getRightVelocity() {
        return RPM.of(leftMotor.get());
    }




}
