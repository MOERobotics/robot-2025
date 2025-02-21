package frc.robot.subsystem.interfaces;


import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface SwerveModuleControl {
    @AutoLog
    class SwerveModuleInputs {
        public Angle currentRotationDegrees = Degrees.zero();
        public Angle currentRotationDegreesNotWrapped = Degrees.zero();
        public double pivotPower;
        public double drivePower;
        public double error, integral;
        public Voltage pivotVolts = Volts.zero();
        public Voltage driveVolts = Volts.zero();
        /**Velocity of the pivot motor*/
        public AngularVelocity pivotVelocity = RPM.zero();
        public AngularVelocity wheelPivotVelocity = RPM.zero();
        public LinearVelocity driveSpeedDesired = MetersPerSecond.zero();
        public Angle drivePosition = Rotations.zero();
        public AngularVelocity driveVelocity = RPM.zero();
        public Angle targetHeading = Radians.zero();
    }
    SwerveModuleInputsAutoLogged getSensors();

    void drive(double power);

    void pivot(Angle heading);

    SwerveModulePosition getModulePosition();

    SwerveModuleState getModuleState();

    void setModuleState(SwerveModuleState moduleState) ;


}
