package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.playingwithfusion.TimeOfFlight;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveDriveControl;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

public class LidarTest extends Command {
    private TimeOfFlight tof_sensor_center = new TimeOfFlight(41);
    private TimeOfFlight tof_sensor_right = new TimeOfFlight(42);
    private TimeOfFlight tof_sensor_left = new TimeOfFlight(40);
    SwerveDriveControl swerveDrive;



    // Called when the command is initially scheduled.

    public LidarTest(SwerveDriveControl swerveDrive){
        this.swerveDrive = swerveDrive;
//        addRequirements(swerveDrive);
    }

    //@Override
   /* public boolean runsWhenDisabled() {
        return true;
    }*/

    @Override
    public void initialize() {
        tof_sensor_center.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        tof_sensor_left.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        tof_sensor_right.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        tof_sensor_center.setRangeOfInterest(6,10,10,6);
        tof_sensor_left.setRangeOfInterest(6,10,10,6);
        tof_sensor_right.setRangeOfInterest(6,10,10,6);
        SmartDashboard.putString("lidar started", "y");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putString("lidar started", "started");
        double left_dist = tof_sensor_left.getRange();
        double center_dist = tof_sensor_center.getRange();
        double right_dist = tof_sensor_right.getRange();

        SmartDashboard.putString("status", String.valueOf(tof_sensor_center.getStatus()));

        SmartDashboard.putNumber("Left Distance", left_dist);
        SmartDashboard.putNumber("Center Distance", center_dist);
        SmartDashboard.putNumber("Right Distance", right_dist);



        if (right_dist < 150){
            SmartDashboard.putString("MOVEMENT", "MOVE LEFT");
            swerveDrive.drive(0, 0, 0);
        } else if (left_dist < 150){
            SmartDashboard.putString("MOVEMENT", "MOVE RIGHT");
            swerveDrive.drive(0, 0, 0);
        } else if (center_dist < 150 && left_dist > 150 && right_dist > 150){
            SmartDashboard.putString("MOVEMENT", "DROP");
            swerveDrive.drive(0, 0, 0);
        } else {
            SmartDashboard.putString("MOVEMENT", "n/a");
            swerveDrive.drive(0, 0, 0);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("lidar test ended", "" + interrupted);
        swerveDrive.drive(0,0,0);
    }
}