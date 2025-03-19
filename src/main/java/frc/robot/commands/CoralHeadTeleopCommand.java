package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.CoralHeadControl;
import frc.robot.subsystem.interfaces.ElevatorControl;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight.LEVEL1;

public class CoralHeadTeleopCommand extends Command {
    CoralHeadControl coralCollector;
    Joystick joystick;
    ElevatorControl elevator;
    boolean hasCoral;
    boolean stopCoral = false;


    public CoralHeadTeleopCommand(RobotContainer robot, Joystick joystick) {
        this.coralCollector = robot.getCoralHead();
        this.joystick = joystick;
        this.elevator = robot.getElevator();
        this.addRequirements(coralCollector);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("DROP",false);
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
            // when at L1 we need to eject differently because the coral must be sideways

            coralWheelRVelocity = RPM.of(0.80);
            coralWheelLVelocity = RPM.of(0.80);


            if (elevator.getHeight().lt(Centimeters.of(70))) {
                coralWheelRVelocity = RPM.of(1.00);
                coralWheelLVelocity = RPM.of(0.30);
            }
        }
        //intake the coral
        else if (joystick.getRawAxis(2) > 0.5) {
            if (!stopCoral) {
                coralWheelLVelocity = RPM.of(0.30);
                coralWheelRVelocity = RPM.of(0.30);
            } else {
                coralWheelRVelocity = RPM.zero();
                coralWheelLVelocity = RPM.zero();
            }
        } else if (joystick.getRawButton(7)) {
            coralWheelRVelocity = RPM.of(-0.30);
            coralWheelLVelocity = RPM.of(-0.30);
        }else {
            hasCoral = false;
            stopCoral = false;
            coralWheelRVelocity = RPM.zero();
            coralWheelLVelocity = RPM.zero();
        }

        coralCollector.setCoralVelocity(coralWheelLVelocity, coralWheelRVelocity);
//        boolean reefLockedOn = coralCollector.inFrontReef() && elevator.getHeight().gt(LEVEL1.measure);
//        SmartDashboard.putBoolean("DROP", reefLockedOn);
        joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        if (elevator.getHeight().gt(LEVEL1.measure)) {
            if (coralCollector.getLeftLidar()){
                joystick.setRumble(GenericHID.RumbleType.kLeftRumble,0.5);
                elevator.setLEDPattern(LEDPattern.solid(Color.kBlue));
            }else if (coralCollector.getRightLidar()) {
                joystick.setRumble(GenericHID.RumbleType.kRightRumble,0.5);
                elevator.setLEDPattern(LEDPattern.solid(Color.kRed));
            }else if(coralCollector.getCenterLidar()){
                joystick.setRumble(GenericHID.RumbleType.kBothRumble, 1);
                elevator.setLEDPattern(LEDPattern.solid(Color.kWhite));
                SmartDashboard.putBoolean("DROP", true);
            }
        }
        if (stopCoral&&!coralCollector.inFrontReef()) {
            elevator.setLEDPattern(LEDPattern.solid(Color.kWhite));
        }


        Logger.recordOutput("CoralHeadTeleop/HasCoral", hasCoral);
        Logger.recordOutput("CoralHeadTeleop/stopCoral", stopCoral);
    }

    @Override
    public void end(boolean interrupted) {
        joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        SmartDashboard.putBoolean("DROP", false);
    }
}


