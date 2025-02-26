package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.CoralHeadControl;
import frc.robot.subsystem.interfaces.ElevatorControl;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.RPM;

public class CoralHeadTeleopCommand extends Command {
    CoralHeadControl coralCollector;
    Joystick joystick;
    ElevatorControl elevator;
    PowerDistribution pdh;
    boolean hasCoral;
    boolean stopCoral = false;


    public CoralHeadTeleopCommand(CoralHeadControl coralCollector, Joystick joystick, ElevatorControl elevator, PowerDistribution pdh) {
        this.coralCollector = coralCollector;
        this.joystick = joystick;
        this.elevator = elevator;
        this.pdh = pdh;
        this.addRequirements(coralCollector);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("DROP",false);
        pdh.setSwitchableChannel(false);
        joystick.setRumble(GenericHID.RumbleType.kBothRumble,0);
    }


    @Override
    public void execute() {
        if (coralCollector.hasCoral()) {
            hasCoral = true;
        }
        if (!coralCollector.hasCoral() && hasCoral) {
            stopCoral = true;
        }
        AngularVelocity coralWheelRVelocity, coralWheelLVelocity;
        //eject the coral
        if (joystick.getRawAxis(3) > 0.5) {
            hasCoral = false;
            stopCoral = false;
            // when at L1 we need to eject differently because the coral must be sideways

            coralWheelRVelocity = RPM.of(0.80);
            coralWheelLVelocity = RPM.of(0.80);


            if (elevator.getHeight().lt(Centimeters.of(70))) {
                coralWheelRVelocity = RPM.of(1.00);
                coralWheelLVelocity = RPM.of(0.30);
            }
        }
        //intake the coral
        else if (joystick.getRawAxis(2) > 0.5 && !stopCoral) {
            coralWheelLVelocity = RPM.of(0.30);
            coralWheelRVelocity = RPM.of(0.30);
        } else if (joystick.getRawButton(7)) {
            coralWheelRVelocity = RPM.of(-0.30);
            coralWheelLVelocity = RPM.of(-0.30);
        } else {
            coralWheelRVelocity = RPM.zero();
            coralWheelLVelocity = RPM.zero();
        }

        coralCollector.setCoralVelocity(coralWheelLVelocity, coralWheelRVelocity);
        if (coralCollector.inFrontReef() && elevator.getHeight().gt(Centimeters.of(80))) {
            SmartDashboard.putBoolean("DROP", true);
            joystick.setRumble(GenericHID.RumbleType.kBothRumble, 1);
            pdh.setSwitchableChannel(true);
        } else {
            SmartDashboard.putBoolean("DROP", false);
            joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            pdh.setSwitchableChannel(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        pdh.setSwitchableChannel(false);
        joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        SmartDashboard.putBoolean("DROP", false);
    }
}


