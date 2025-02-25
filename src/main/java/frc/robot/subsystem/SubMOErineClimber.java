package frc.robot.subsystem;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ClimberControl;
import frc.robot.subsystem.interfaces.ClimberInputsAutoLogged;

import static edu.wpi.first.units.Units.*;

public class SubMOErineClimber extends MOESubsystem<ClimberInputsAutoLogged> implements ClimberControl {

    public SparkMax climbMotor;
    public CANcoder climbEncoder;

    public final Angle maxEncoderValue = Degrees.of(72);
    public final Angle minEncoderValue = Degrees.of(0);

    public SubMOErineClimber(
        SparkMax climbMotor,
        CANcoder climbEncoder,
        String name
    ) {
        super(new ClimberInputsAutoLogged(), name);
        this.climbMotor = climbMotor;
        this.climbEncoder = climbEncoder;
    }

    @Override
    public void readSensors(ClimberInputsAutoLogged sensors) {
        sensors.motorPower = climbMotor.get();
        sensors.motorAppliedVolts = Volts.of(climbMotor.getAppliedOutput()*climbMotor.getBusVoltage());
        sensors.position = climbEncoder.getAbsolutePosition().getValue();
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
