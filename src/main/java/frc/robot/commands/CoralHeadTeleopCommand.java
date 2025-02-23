package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystem.interfaces.CoralHeadControl;
import frc.robot.subsystem.interfaces.ElevatorControl;

import static edu.wpi.first.units.Units.*;

public class CoralHeadTeleopCommand extends Command {
    CoralHeadControl coralCollector;
    Joystick joystick;
    ElevatorControl elevator;
    boolean hasCoral;
    boolean stopCoral = false;


    public CoralHeadTeleopCommand(CoralHeadControl coralCollector, Joystick joystick, ElevatorControl elevator) {
        this.coralCollector = coralCollector;
        this.joystick = joystick;
        this.elevator = elevator;
        this.addRequirements(coralCollector);
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        if(coralCollector.hasCoral()) {
            hasCoral = true;
        }
        if(!coralCollector.hasCoral() && hasCoral){
            stopCoral = true;
        }
        AngularVelocity coralWheelRVelocity, coralWheelLVelocity;
        //eject the coral
        if (joystick.getRawAxis(2)>0.5){
           hasCoral = false;
           stopCoral = false;
           // when at L1 we need to eject differently because the coral must be sideways

            coralWheelRVelocity = RPM.of(0.80);
            coralWheelLVelocity = RPM.of(0.80);


           if (elevator.getHeight().lt(Inches.of(45))){
               coralWheelRVelocity = RPM.of(1.00);
               coralWheelLVelocity = RPM.of(0.30);
           }
        }
        //intake the coral
        else if(joystick.getRawAxis(3)> 0.5 && !stopCoral) {
            coralWheelLVelocity = RPM.of(0.30);
            coralWheelRVelocity = RPM.of(0.30);
        } else {
            coralWheelRVelocity = RPM.zero();
            coralWheelLVelocity = RPM.zero();
        }

        /* else if(joystick.getRawAxis(1)>0.5){
            coralWheelRVelocity = RPM.of(-0.80);
            coralWheelLVelocity = RPM.of(-0.80);


        }*/

        coralCollector.setCoralVelocity(coralWheelLVelocity, coralWheelRVelocity);


    }
}


