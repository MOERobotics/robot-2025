package frc.robot.subsystem;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class LimeLights extends Command {
    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)

        boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

        double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC("");  // Vertical  offset from principal pixel/point to target in degrees
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        if(limelightMeasurement != null) {
            Logger.recordOutput("LimeMeasurement", limelightMeasurement.pose);
            Logger.recordOutput("tx", tx);
            Logger.recordOutput("tx", ty);
            Logger.recordOutput("tx", ta);
            Logger.recordOutput("tx", txnc);
            Logger.recordOutput("tx", tync);
        }
    }
}
