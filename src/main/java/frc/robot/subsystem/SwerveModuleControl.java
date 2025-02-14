package frc.robot.subsystem;


import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleControl {
    @AutoLog
    public static class SwerveModuleInputs {
        public Angle currentRotationDegrees;
        public double pivotPower;
        public double drivePower;
        public double error, integral;

        public Angle targetHeading;
    }
    SwerveModuleInputsAutoLogged getSensors();

    void drive(double power);

    void pivot(Angle heading);

    SwerveModulePosition getModulePosition();
    SwerveModuleState getModuleState();

}
