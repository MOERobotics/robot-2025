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
        sensors.canGoDown = canGoDown();
        sensors.canGoUp = canGoUp();
        sensors.position = getPosition();
        sensors.motorVelocity = getMotorVelocity();



    }
    @Override
    public boolean canGoUp() {
        return getPosition().gt(minEncoderValue);
    }

    @Override
    public boolean canGoDown() {
        return getPosition().lt(maxEncoderValue);
    }

    @Override
    public Angle getPosition(){
        return climbEncoder.getAbsolutePosition().getValue();
    }


    @Override
    public AngularVelocity getMotorVelocity() {
        return RPM.of(climbMotor.getEncoder().getVelocity());
    }

    public void setTestClimberVelocity(AngularVelocity power) {
            climbMotor.set(power.in(RPM));
    }
    @Override
    public void setClimberVelocity(AngularVelocity power){
            if (power.in(RPM) > 0 && canGoUp()) {
                climbMotor.set(power.in(RPM));

            } else if (power.in(RPM) < 0 && canGoDown()) {
                climbMotor.set(power.in(RPM));
            } else{
               climbMotor.set(0);
            }

        }









}
