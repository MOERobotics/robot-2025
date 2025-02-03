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
        double sum = 0;
        double deviationSum = 0;

        for (int i = 0; i < poses.size(); i++) {
            sum += poses.get(i).getX();
        }

        double mean = sum / poses.size();

        for (int i = 0; i < poses.size(); i++) {
            double deviation = poses.get(i).getX() - mean;
            deviation = Math.pow(deviation, 2);
            deviationSum += deviation;
        }

        double variance = deviationSum / poses.size();
        double standardDev = Math.pow(variance, 0.5);

        // return standardDev;
    }
}