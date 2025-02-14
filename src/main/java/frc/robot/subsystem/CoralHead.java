package frc.robot.subsystem;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.MOESubsystem;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RPM;

public class CoralHead extends MOESubsystem<CoralHeadInputsAutoLogged> implements CoralHeadControl {

    public SparkMax leftMotor ;
    public SparkMax rightMotor;



    public CoralHead(SparkMax leftMotor, SparkMax rightMotor){
        this.setSensors(new CoralHeadInputsAutoLogged());
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        getSensors().hasCoral = false;
        getSensors().inFrontReef = false;
        getSensors().velocityRight = RPM.zero();
        getSensors().velocityLeft = RPM.zero();
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.inverted(true);
        leftMotor.configure(leftConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        //leftMotor.setInverted(true);
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
    public void readSensors(CoralHeadInputsAutoLogged sensors) {
        sensors.hasCoral = leftMotor.getReverseLimitSwitch().isPressed();
        sensors.inFrontReef = false;
        sensors.velocityLeft = RPM.of(leftMotor.getEncoder().getVelocity());
        sensors.velocityRight = RPM.of(rightMotor.getEncoder().getVelocity());
    }


    @Override
    public void setCoralVelocity(AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity) {
        leftMotor.set(leftAngularVelocity.in(RPM));
        rightMotor.set(rightAngularVelocity.in(RPM));
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
