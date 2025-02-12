package frc.robot.subsystem;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class SubMOErineClimber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberControl {

    SparkMax climbMotor;

    CANcoder climbEncoder;



    public final double maxEncoderValue = 0;
    public final double minEncoderValue = 0;


    public double tolerance =5;



    public SubMOErineClimber(
            SparkMax climbMotor,
            CANcoder climbEncoder
    ) {
        this.setSensors(new ClimberInputsAutoLogged());
        this.climbMotor = climbMotor;
        this.climbEncoder = climbEncoder;
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.inverted(false).idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);
        climbMotor.configure(climberConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void readSensors(ClimberInputsAutoLogged sensors) {
        sensors.canGoDown = canGoDown();
        sensors.canGoUp = canGoUp();
        sensors.position = getPosition();
        sensors.motorVelocity = getMotorVelocity();


    }
    @Override
    public boolean canGoUp() {
        return !(getPosition().in(Degree) + tolerance> maxEncoderValue);
    }

    @Override
    public boolean canGoDown() {
        return !(getPosition().in(Degree) + tolerance < minEncoderValue);
    }




    @Override
    public Angle getPosition(){ return climbEncoder.getAbsolutePosition().getValue(); }


    @Override
    public AngularVelocity getMotorVelocity() {
        return RPM.of(climbMotor.getEncoder().getVelocity());
    }



    @Override
    public void setClimberVelocity(AngularVelocity power) {
       if (power.in(RPM)> 0 && canGoUp()) {
           climbMotor.set(power.in(RPM));

       } else if(power.in(RPM) < 0 && canGoDown()){
           climbMotor.set(power.in(RPM));
       }else{
           climbMotor.set(0);
       }


}







}
