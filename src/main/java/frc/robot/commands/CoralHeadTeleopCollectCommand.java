package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.CoralHeadControl;

import static edu.wpi.first.units.Units.*;

/**
 * An example command that uses an example subsystem.
 */
public class CoralHeadTeleopCollectCommand extends Command {

    CoralHeadControl coralHeadControl;


    Joystick joystick;

    AngularVelocity leftVelocity;
    AngularVelocity rightVelocity;


    int time;
    boolean hasCollected;



    public CoralHeadTeleopCollectCommand(CoralHeadControl coralHeadControl, Joystick joystick) {
        this.coralHeadControl = coralHeadControl;
        this.joystick = joystick;

        addRequirements(coralHeadControl);


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftVelocity = RPM.zero();
        rightVelocity = RPM.zero();
        time =0;
        hasCollected = false;

    }


    //TODO update magic numbers for velocities
    @Override
    public void execute() {
      int currTime;



        if (!hasCollected) {

            if(joystick.getRawButton(1)){
                leftVelocity = RPM.of(1);
                rightVelocity = RPM.of(1);
            } else{
                leftVelocity = RPM.of(0);
                rightVelocity = RPM.of(0);
            }

            if(coralHeadControl.hasCoral()) {
                hasCollected = true;
                leftVelocity = RPM.of(0);
                rightVelocity = RPM.of(0);
            }





        } else {
            time++;
        }



        coralHeadControl.setCoralVelocity(leftVelocity, rightVelocity);



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralHeadControl.setCoralVelocity(RPM.zero(), RPM.zero());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return hasCollected && time>=3;
    }
}