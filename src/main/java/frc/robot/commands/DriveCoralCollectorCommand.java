package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.CoralCollectorControl;

import static edu.wpi.first.units.Units.*;

/**
 * An example command that uses an example subsystem.
 */
public class DriveCoralCollectorCommand extends Command {

    CoralCollectorControl coralCollectorControl;


    Joystick joystick;

    AngularVelocity leftVelocity;
    AngularVelocity rightVelocity;


    public DriveCoralCollectorCommand(CoralCollectorControl coralCollectorControl, Joystick joystick) {
        this.coralCollectorControl = coralCollectorControl;
        this.joystick = joystick;

        addRequirements(coralCollectorControl);


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftVelocity = RPM.of(0);
        rightVelocity = RPM.of(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        /* TODO UPDATE VELOCITIES FOR L1*/
        if (joystick.getRawButton(1)) {
            leftVelocity = RPM.of(1);
            rightVelocity = RPM.of(1);
        }else if(joystick.getRawButton(2)) {

            leftVelocity = RPM.of(0.75);
            rightVelocity = RPM.of(1);

        }
        else {
            leftVelocity = RPM.of(0);
            rightVelocity = RPM.of(0);
        }

        coralCollectorControl.setCoralVelocity(leftVelocity, rightVelocity);



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}