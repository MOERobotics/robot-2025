package frc.robot.subsystem;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.RPM;

public class CoralCollector extends MOESubsystem<CoralCollectorInputsAutoLogged> implements CoralCollectorControl {

    public SparkMax leftMotor ;
    public SparkMax rightMotor;



    public CoralCollector(SparkMax leftMotor, SparkMax rightMotor){
        this.setSensors(new CoralCollectorInputsAutoLogged());
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
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
