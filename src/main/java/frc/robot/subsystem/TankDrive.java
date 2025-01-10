package frc.robot.subsystem;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class TankDrive extends SubsystemBase {
    TankDriveIO io;
    TankDriveInputsAutoLogged inputs = new TankDriveInputsAutoLogged();

    var sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,null,null,
                    (state)-> Logger.recordOutput("SysIDState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (voltage)-> io.setVoltageLeft(V)
            )
    )
    public TankDrive(TankDriveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("TankDrive", this.inputs);
    }

    public void drive(double left, double right) {
        this.io.drive(left, right);
    }

    public void stop() {


        this.io.drive(0,0);

    }


    public double getLeftDist(){
        return inputs.encoderTicksLeft;
    }


    public double getInches(){
        return inputs.encoderTicksLeft/112.0;
    }


    public double getAngle(){
        return inputs.navxYaw;

    }

}