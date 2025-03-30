package frc.robot.subsystem.interfaces;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface CoralHeadControl extends Subsystem {

    @AutoLog
    class CoralHeadInputs {
        public boolean frontBeam;
        public boolean backBeam;
        public boolean inFrontReef;
        public boolean leftLidarOn, rightLidarOn, centerLidarOn;
        public double reefDistance;
        public double leftPower;
        public double rightPower;
        public AngularVelocity velocityLeft = RPM.zero();
        public AngularVelocity velocityRight = RPM.zero();
        public Voltage leftAppliedVolts = Volts.zero();
        public Voltage rightAppliedVolts = Volts.zero();

    }
    CoralHeadInputsAutoLogged getSensors();

    default boolean frontBeam(){
        return this.getSensors().frontBeam;
    }
    default boolean backBeam(){return this.getSensors().backBeam;}

    default boolean inFrontReef(){
        return this.getSensors().inFrontReef;
    }

    default boolean getLeftLidar() {
        return this.getSensors().leftLidarOn;
    }
    default boolean getCenterLidar() {
        return this.getSensors().centerLidarOn;
    }
    default boolean getRightLidar() {
        return this.getSensors().rightLidarOn;
    }

    void setCoralVelocity(AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity);

    default AngularVelocity getLeftVelocity(){
        return this.getSensors().velocityLeft;
    }

    default AngularVelocity getRightVelocity(){
        return this.getSensors().velocityRight;
    }
}
