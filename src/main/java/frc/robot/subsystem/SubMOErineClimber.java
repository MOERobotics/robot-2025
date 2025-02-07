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

    SparkMax climbTopMotor;

    SparkMax climbBottomMotor;

    CANcoder climbTopEncoder;

    CANcoder climbBottomEncoder;


    public final double maxTopEncoderValue = 0;
    public final double minTopEncoderValue = 0;

    public final double maxBottomEncoderValue = 0;
    public final double minBottomEncoderValue = 0;
    public double tolerance =5;



    public SubMOErineClimber(
            SparkMax climbTopMotor,
            SparkMax climbBottomMotor,
            CANcoder climbTopEncoder,
            CANcoder climbBottomEncoder
    ) {
        this.setSensors(new ClimberInputsAutoLogged());
        this.climbTopMotor = climbTopMotor;
        this.climbTopEncoder = climbTopEncoder;
        this.climbBottomMotor = climbBottomMotor;
        this.climbBottomEncoder = climbBottomEncoder;
        SparkMaxConfig climberTopConfig = new SparkMaxConfig();
        climberTopConfig.inverted(false).idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);
        climbTopMotor.configure(climberTopConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);


        SparkMaxConfig climberBottomConfig = new SparkMaxConfig();
        climberBottomConfig.inverted(false).idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);
        climbBottomMotor.configure(climberTopConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);


    }

    @Override
    public void readSensors(ClimberInputsAutoLogged sensors) {
        sensors.canGoDownTop = canGoDownTop();
        sensors.canGoUpTop = canGoUpTop();
        sensors.topPosition = getTopPosition();
        sensors.topMotorVelocity = getTopMotorVelocity();

        sensors.canGoDownBottom = canGoDownBottom();
        sensors.canGoUpBottom = canGoUpBottom();
        sensors.bottomPosition = getBottomPosition();
        sensors.bottomMotorVelocity = getBottomMotorVelocity();

    }
    @Override
    public boolean canGoUpTop () {
        return !(getTopPosition().in(Degree) + tolerance> maxTopEncoderValue);
    }

    @Override
    public boolean canGoDownTop () {
        return !(getTopPosition().in(Degree) + tolerance < minTopEncoderValue);
    }

    @Override
    public boolean canGoUpBottom () {
        return !(getBottomPosition().in(Degree) + tolerance> maxBottomEncoderValue);
    }

    @Override
    public boolean canGoDownBottom () {
        return !(getBottomPosition().in(Degree) + tolerance < minBottomEncoderValue);
    }


    @Override
    public Angle getTopPosition(){ return climbTopEncoder.getAbsolutePosition().getValue(); }
    @Override
    public Angle getBottomPosition(){ return climbBottomEncoder.getAbsolutePosition().getValue(); }


    @Override
    public AngularVelocity getTopMotorVelocity() {
        return RPM.of(climbTopMotor.getEncoder().getVelocity());
    }

    @Override
    public AngularVelocity getBottomMotorVelocity() {
        return RPM.of(climbBottomMotor.getEncoder().getVelocity());
    }

    @Override
    public void setClimberTopVelocity(AngularVelocity power) {
       if (power.in(RPM)> 0 && canGoUpTop()) {
           climbTopMotor.set(power.in(RPM));

       } else if(power.in(RPM) < 0 && canGoDownTop()){
           climbTopMotor.set(power.in(RPM));
       }



}

    @Override
    public void setClimberBottomVelocity(AngularVelocity power) {
        if (power.in(RPM)> 0 && canGoUpBottom()) {
            climbBottomMotor.set(power.in(RPM));

        } else if(power.in(RPM) < 0 && canGoDownBottom()){
            climbBottomMotor.set(power.in(RPM));
        }



    }



}
