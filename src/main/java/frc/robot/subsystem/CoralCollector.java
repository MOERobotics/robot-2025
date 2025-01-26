package frc.robot.subsystem;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import lombok.Builder;

import static edu.wpi.first.units.Units.*;

public class CoralCollector extends MOESubsystem<CoralCollectorInputsAutoLogged> implements CoralCollectorIO{

    @Builder
    public static record CoralCollectorConstants(int leftMotorID,int rightMotorID,boolean leftInvert, boolean rightInvert,int currentLimit,double kP, double kI, double kD) {}

    GenericMotor leftMotor,rightMotor;
    GenericMotorInputsAutoLogged leftMotorInputs = new GenericMotorInputsAutoLogged();
    GenericMotorInputsAutoLogged rightMotorInputs = new GenericMotorInputsAutoLogged();

    public CoralCollector(CoralCollectorConstants constants){
        this.setSensors(new CoralCollectorInputsAutoLogged());
        this.leftMotor = new GenericSparkMax(constants.leftMotorID);
        this.rightMotor = new GenericSparkMax(constants.rightMotorID);

        leftMotor.setInvert(constants.leftInvert);
        rightMotor.setInvert(constants.rightInvert);

        leftMotor.setCurrentLimit(constants.currentLimit);
        rightMotor.setCurrentLimit(constants.currentLimit);

        leftMotor.setPID(new PIDConstants(constants.kP,constants.kI,constants.kP));
        rightMotor.setPID(new PIDConstants(constants.kP,constants.kI,constants.kP));

        leftMotor.setReverseLimitType(GenericMotor.LimitType.CLOSED);
        leftMotor.setReverseLimitEnabled(false);
    }

    @Override
    public void readSensors(CoralCollectorInputsAutoLogged inputs) {
        leftMotor.processInputs(leftMotorInputs);
        rightMotor.processInputs(rightMotorInputs);
        inputs.hasCoral = hasCoral();
        inputs.velocityLeft = getLeftVelocity();
        inputs.velocityRight = getRightVelocity();
    }

    @Override
    public boolean hasCoral() {
        return leftMotor.getReverseLimit();
    }

    @Override
    public boolean inFrontReef() {
        return false;
    }

    @Override
    public void setCoralVelocity(AngularVelocity leftPower, AngularVelocity rightPower) {
        leftMotor.set(leftPower.in(RotationsPerSecond)/4.0);
        rightMotor.set(rightPower.in(RotationsPerSecond)/4.0);
    }

    @Override
    public AngularVelocity getLeftVelocity() {
        return RotationsPerSecond.of(leftMotor.getVelocity()*60);
    }

    @Override
    public AngularVelocity getRightVelocity() {
        return RotationsPerSecond.of(rightMotor.getVelocity()*60);
    }
}
