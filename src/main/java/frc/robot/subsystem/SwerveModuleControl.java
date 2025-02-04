package frc.robot.subsystem;


import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleControl {
    @AutoLog
    public static class SwerveModuleInputs {
        public Angle currentRotationDegrees;
        public double pivotPower;
        public double drivePower;
        public double error, integral;

    }
    SwerveModuleInputsAutoLogged getSensors();

    void drive(double power);

    void pivot(Angle heading);


}
