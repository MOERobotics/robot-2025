package frc.robot.subsystem;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.CoralHeadControl;
import frc.robot.subsystem.interfaces.CoralHeadInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

public class CoralHead extends MOESubsystem<CoralHeadInputsAutoLogged> implements CoralHeadControl {

    public SparkMax leftMotor;
    public SparkMax rightMotor;
    public TimeOfFlight tofCenter, tofLeft, tofRight;


    public CoralHead(SparkMax leftMotor, SparkMax rightMotor) {
        super(new CoralHeadInputsAutoLogged());
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40).inverted(true);
        leftMotorConfig.limitSwitch.forwardLimitSwitchEnabled(false);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40).inverted(false);

        leftMotor.configure(leftMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        rightMotor.configure(rightMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        this.tofCenter = new TimeOfFlight(41);
        this.tofLeft = new TimeOfFlight(40);
        this.tofRight = new TimeOfFlight(42);
        this.tofCenter.setRangeOfInterest(8, 8, 9, 9);
        this.tofCenter.setRangingMode(TimeOfFlight.RangingMode.Short,24);
        getSensors().hasCoral = false;
        getSensors().inFrontReef = false;
        getSensors().velocityRight = RPM.zero();
        getSensors().velocityLeft = RPM.zero();
    }


    @Override
    public void readSensors(CoralHeadInputsAutoLogged sensors) {
        sensors.hasCoral = leftMotor.getForwardLimitSwitch().isPressed();
        sensors.inFrontReef = tofCenter.isRangeValid() && tofCenter.getRange() < 709.6;
        sensors.reefDistance = tofCenter.getRange();
        sensors.leftPower = leftMotor.get();
        sensors.rightPower = rightMotor.get();
        sensors.velocityLeft = RPM.of(leftMotor.getEncoder().getVelocity());
        sensors.velocityRight = RPM.of(rightMotor.getEncoder().getVelocity());
        sensors.leftAppliedVolts = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
        sensors.rightAppliedVolts = Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
        Logger.recordOutput("LeftLidarRangeValid", tofLeft.isRangeValid());
        Logger.recordOutput("RightLidarRangeValid", tofRight.isRangeValid());
        Logger.recordOutput("LeftLidarRange", tofLeft.getRange());
        Logger.recordOutput("RightLidarRange", tofRight.getRange());
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
