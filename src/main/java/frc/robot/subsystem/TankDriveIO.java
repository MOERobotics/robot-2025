package frc.robot.subsystem;

import org.littletonrobotics.junction.AutoLog;

public interface TankDriveIO {
    @AutoLog
    public class TankDriveInputs {
        public int encoderTicksLeft;
        public int encoderTicksRight;
//        public double navxYaw;
        public double motorPosition;

        public double motorVelocity;

        public double appliedVoltage;


    }

    default void updateInputs(TankDriveInputs inputs){}

    default void drive(double leftPower, double rightPower){}

    default void setVoltageLeft(double voltage){

    }
    default void setVoltageRight(double voltage){

    }
}