package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.List;

public class standardDeviation extends Command {
    ArrayList<Pose2d> poses = new ArrayList<>(10);

    public Pose2d getPose() {
        return null;
    }

    @Override
    public void initialize() {
        poses.clear();
    }

    @Override
    public void execute() {
        var pose = getPose();
        poses.add(pose);
    }

    @Override
    public void end(boolean interrupted) {
        double sumX = 0;
        double sumY = 0;
        double sumR = 0;

        double deviationSumX = 0;
        double deviationSumY = 0;
        double deviationSumR = 0;

        for (int i = 0; i < poses.size(); i++) {
            sumX += poses.get(i).getX();
            sumY += poses.get(i).getY();
            sumR += poses.get(i).getRotation();
        }

        double meanX = sumX / poses.size();
        double meanY = sumY / poses.size();
        double meanR = sumR / poses.size();

        for (int i = 0; i < poses.size(); i++) {
            double deviationX = poses.get(i).getX() - meanX;
            deviationX = Math.pow(deviationX, 2);
            deviationSumX += deviationX;

            double deviationY = poses.get(i).getY() - meanY;
            deviationY = Math.pow(deviationY, 2);
            deviationSumY += deviationY;

            double deviationR = poses.get(i).getRotation() - meanR;
            deviationR = Math.pow(deviationR, 2);
            deviationSumR += deviationR;
        }

        double varianceX = deviationSumX / poses.size();
        double standardDevX = Math.pow(varianceX, 0.5);

        double varianceY = deviationSumY / poses.size();
        double standardDevY = Math.pow(varianceY, 0.5);

        double varianceR = deviationSumR / poses.size();
        double standardDevR = Math.pow(varianceR, 0.5);

        // return standardDev;
    }
}