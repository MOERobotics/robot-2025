package frc.robot.subsystem;


import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ClimberControl;
import frc.robot.subsystem.interfaces.ClimberInputsAutoLogged;

import static edu.wpi.first.units.Units.*;

public class SubMOErineClimber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberControl {

    public SparkMax climbMotor;
    public SparkAbsoluteEncoder climbEncoder;

    public final Angle maxEncoderValue = Degrees.of(80);
    public final Angle minEncoderValue = Degrees.of(-7);


    public SubMOErineClimber(
        SparkMax climbMotor,
        boolean inverted,
        String name
    ) {
        super(new ClimberInputsAutoLogged(), name);
        this.climbMotor = climbMotor;
        this.climbEncoder = climbMotor.getAbsoluteEncoder();
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(inverted).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);
        sparkMaxConfig.absoluteEncoder.inverted(!inverted);
        climbMotor.configure(sparkMaxConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void readSensors(ClimberInputsAutoLogged sensors) {
        sensors.motorPower = climbMotor.get();
        sensors.motorAppliedVolts = Volts.of(climbMotor.getAppliedOutput()*climbMotor.getBusVoltage());
        sensors.position = Rotations.of(MathUtil.inputModulus(climbEncoder.getPosition(),-0.5,0.5));
        sensors.motorVelocity = RPM.of(climbMotor.getEncoder().getVelocity());
        sensors.canGoDown = getPosition().gt(minEncoderValue);
        sensors.canGoUp = getPosition().lt(maxEncoderValue);
    }

    public void setTestClimberVelocity(AngularVelocity power) {
            climbMotor.set(power.in(RPM));
    }

    @Override
    public void setClimberVelocity(AngularVelocity power){
            if ((power.lt(RPM.zero()) && canGoDown())||(power.gt(RPM.zero()) && canGoUp())) {
                climbMotor.set(power.in(RPM));
            } else{
               climbMotor.set(0);
            }

        }









}
