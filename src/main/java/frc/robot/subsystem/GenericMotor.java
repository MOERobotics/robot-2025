package frc.robot.subsystem;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface GenericMotor {
    @AutoLog
    class GenericMotorInputs{
        Voltage appliedVolts;
        AngularVelocity motorVelocity;
        Angle motorPosition;
        boolean motorInvert;
        double kP;
        double kI;
        double kD;
    }

    void processInputs(GenericMotorInputs inputs);

    enum IdleMode{
        BRAKE,
        COAST
    }
    void setIdleMode(IdleMode idleMode);

    void set(double power);

    double get();

    void setVelocity(AngularVelocity angularVelocity);

    double getVelocity();

    double getPosition();

    void stopMotor();

    void setPID(PIDConstants pidConstants);

    PIDConstants getPID();

    void setInvert(boolean invert);

    boolean getInvert();

    default void setCurrentLimit(int currentLimit){
        setCurrentLimit(currentLimit,0);
    };

    void setCurrentLimit(int stallLimit, int freeLimit);

    int getID();

    GenericMotor withPID(PIDConstants pidConstants);

    GenericMotor withInvert(boolean invert);

    default GenericMotor withCurrentLimit(int currentLimit){
        return withCurrentLimit(currentLimit,0);
    };

    GenericMotor withCurrentLimit(int stallLimit, int freeLimit);

    enum LimitType{
        OPEN,
        CLOSED,
    }

    void setForwardLimitType(LimitType limitType);

    void setReverseLimitType(LimitType limitType);

    void setForwardLimitEnabled(boolean enabled);

    void setReverseLimitEnabled(boolean enabled);

    boolean getForwardLimit();

    boolean getReverseLimit();

    default void setFollower(int leadID){
        setFollower(leadID,false);
    }

    void setFollower(int leadID, boolean invert);

    int getLead();
}
