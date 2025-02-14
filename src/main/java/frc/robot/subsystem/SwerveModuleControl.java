package frc.robot.subsystem;


import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleControl {
    @AutoLog
    public static class SwerveModuleInputs {
        public Angle currentRotationDegrees;
        public Angle currentRotationDegreesNotWrapped;
        public double pivotPower;
        public double drivePower;
        public double error, integral;
        public Voltage pivotVolts;
        public Voltage driveVolts;
        /**Velocity of the pivot motor*/
        public AngularVelocity pivotVelocity;
        public AngularVelocity wheelPivotVelocity;
        public LinearVelocity driveSpeedDesired;
        public Angle drivePosition;
        public AngularVelocity driveVelocity;
        public Angle targetHeading;
    }
    SwerveModuleInputsAutoLogged readSensors();

    void drive(double power);

    void pivot(Angle heading);

    void setModuleState(SwerveModuleState moduleState);

    SwerveModulePosition getModulePosition();

    SwerveModuleState getModuleState();

}
