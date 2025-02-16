package frc.robot;
//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.LimelightHelpers;

import java.util.ArrayList;

public class standardDeviation extends Command {
    ArrayList<Pose2d> poses = new ArrayList(10);

    public standardDeviation() {
    }

    public Pose2d getPose() {
        return LimelightHelpers.getBotPose3d("limelight").toPose2d();
    }

    public void initialize() {
        this.poses.clear();
    }

    public void execute() {
        Pose2d pose = this.getPose();
        this.poses.add(pose);
    }

    public void end(boolean interrupted) {
        double sum = (double)0.0F;
        double deviationSum = (double)0.0F;

        for(int i = 0; i < this.poses.size(); ++i) {
            sum += ((Pose2d)this.poses.get(i)).getX();
        }

        double mean = sum / (double)this.poses.size();

        for(int i = 0; i < this.poses.size(); ++i) {
            double deviation = ((Pose2d)this.poses.get(i)).getX() - mean;
            deviation = Math.pow(deviation, (double)2.0F);
            deviationSum += deviation;
        }

        double variance = deviationSum / (double)this.poses.size();
        double standardDev = Math.pow(variance, (double)0.5F);
    }
}
