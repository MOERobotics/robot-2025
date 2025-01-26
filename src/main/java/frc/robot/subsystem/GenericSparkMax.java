package frc.robot.subsystem;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class GenericSparkMax implements GenericMotor{
    SparkMax motor;
    SparkClosedLoopController motorController;
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    public GenericSparkMax(int motorID){
        motor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public void applyConfig(){
        motor.configure(motorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }


    @Override
    public void processInputs(GenericMotorInputs inputs) {
        inputs.appliedVolts = Volts.of(motor.getAppliedOutput()*motor.getBusVoltage());
        inputs.motorVelocity = RPM.of(getVelocity());
        inputs.motorPosition = Revolutions.of(getPosition());
        inputs.motorInvert = getInvert();
        inputs.kP = getPID().kP;
        inputs.kI = getPID().kI;
        inputs.kD = getPID().kD;
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        switch (idleMode){
            case BRAKE -> motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
            case COAST -> motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        }
        applyConfig();
    }

    @Override
    public void set(double power) {
        motor.set(power);
    }

    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public void setVelocity(AngularVelocity angularVelocity) {
        motorController.setReference(angularVelocity.in(RPM), SparkBase.ControlType.kVelocity);
    }

    @Override
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void setPID(PIDConstants pidConstants) {
        motorConfig.closedLoop.pid(pidConstants.kP,pidConstants.kI,pidConstants.kD).iZone(pidConstants.iZone);
        applyConfig();
    }

    @Override
    public PIDConstants getPID() {
        return new PIDConstants(motor.configAccessor.closedLoop.getP(),motor.configAccessor.closedLoop.getI(),motor.configAccessor.closedLoop.getD(),motor.configAccessor.closedLoop.getIZone());
    }

    @Override
    public void setInvert(boolean invert) {
        motorConfig.inverted(invert);
        applyConfig();
    }

    @Override
    public boolean getInvert() {
        return motor.configAccessor.getInverted();
    }

    @Override
    public void setCurrentLimit(int currentLimit) {
        this.setCurrentLimit(currentLimit,0);
    }

    @Override
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit,freeLimit);
        applyConfig();
    }

    @Override
    public int getID() {
        return motor.getDeviceId();
    }

    @Override
    public GenericMotor withPID(PIDConstants pidConstants) {
        setPID(pidConstants);
        return this;
    }

    @Override
    public GenericMotor withInvert(boolean invert) {
        setInvert(invert);
        return this;
    }

    @Override
    public GenericMotor withCurrentLimit(int currentLimit) {
        return withCurrentLimit(currentLimit,0);
    }

    @Override
    public GenericMotor withCurrentLimit(int stallLimit, int freeLimit) {
        setCurrentLimit(stallLimit,freeLimit);
        return this;
    }

    @Override
    public void setForwardLimitType(LimitType limitType) {
        switch (limitType){
            case OPEN -> motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
            case CLOSED -> motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        }
        applyConfig();
    }

    @Override
    public void setReverseLimitType(LimitType limitType) {
        switch (limitType){
            case OPEN -> motorConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
            case CLOSED -> motorConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        }
        applyConfig();
    }

    @Override
    public void setForwardLimitEnabled(boolean enabled) {
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(enabled);
        applyConfig();
    }

    @Override
    public void setReverseLimitEnabled(boolean enabled) {
        motorConfig.limitSwitch.reverseLimitSwitchEnabled(enabled);
        applyConfig();
    }

    @Override
    public boolean getForwardLimit() {
        return motor.getForwardLimitSwitch().isPressed();
    }

    @Override
    public boolean getReverseLimit() {
        return motor.getReverseLimitSwitch().isPressed();
    }

    @Override
    public void setFollower(int leadID) {
        setFollower(leadID,false);
    }

    @Override
    public void setFollower(int leadID, boolean invert) {
        motorConfig.follow(leadID,invert);
        applyConfig();
    }

    @Override
    public int getLead() {
        return motor.configAccessor.getFollowerModeLeaderId();
    }
}
