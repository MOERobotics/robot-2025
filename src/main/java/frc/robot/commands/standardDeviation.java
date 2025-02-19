package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Random;

import static edu.wpi.first.math.MathUtil.inputModulus;

public class standardDeviation extends Command {
    ArrayList<Pose3d> poses = new ArrayList<>(10);
    Random rand = new Random();

    public Pose3d getPose() {
       // double x = rand.nextGaussian(5, 2);
       // double y = rand.nextGaussian(5, 2);
       // double theta = rand.nextGaussian(Units.degreesToRadians(90), Units.degreesToRadians(20));

        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
        double tyaw = LimelightHelpers.getBotPose2d("").getRotation().getDegrees();  // Target area (0% to 100% of image)

        // Do I need to add the tyaw == 0? Gives an error when tried.
        if (tx == 0 || ty == 0) {
            return null;
        } else {
            return new Pose3d(
                    new Translation3d(tx, ty, 0),
                    new Rotation3d(0, 0, tyaw)
            );
        }
    }

    @Override
    public void initialize() {
        poses.clear();
    }

    @Override
    public void execute() {
        var pose = getPose();
        if (pose == null) return;
        else poses.add(pose);
    }

    @Override
    public void end(boolean interrupted) {
        ArrayList<Double> rotation = new ArrayList<>();

        double sumX = 0;
        double sumY = 0;
        double sumR = 0;

        double deviationSumX = 0;
        double deviationSumY = 0;
        double deviationSumR = 0;

        for (int i = 0; i < poses.size(); i++) {
            sumX += poses.get(i).getX();
            sumY += poses.get(i).getY();

            Rotation3d firstTerm = poses.get(0).getRotation();
            Rotation3d currentTerm = poses.get(i).getRotation();
            double firstAngle = firstTerm.toRotation2d().getDegrees();
            double currentAngle = currentTerm.toRotation2d().getDegrees();

            double convertedAngle = inputModulus(firstAngle - currentAngle, -180, 180);
            rotation.add(convertedAngle);
            sumR += convertedAngle;
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

            double deviationR = rotation.get(i) - meanR;
            deviationR = Math.pow(deviationR, 2);
            deviationSumR += deviationR;
        }

        double varianceX = deviationSumX / poses.size();
        double standardDevX = Math.pow(varianceX, 0.5);
        // Logger.recordOutput("X standard deviation", standardDevX);
        SmartDashboard.putNumber("X standard dev", standardDevX);

        double varianceY = deviationSumY / poses.size();
        double standardDevY = Math.pow(varianceY, 0.5);
        // Logger.recordOutput("Y standard deviation", standardDevY);
        SmartDashboard.putNumber("Y standard dev", standardDevY);

        double varianceR = deviationSumR / poses.size();
        double standardDevR = Math.pow(varianceR, 0.5);
        SmartDashboard.putNumber("Rotation standard dev", standardDevR);
        // Logger.recordOutput("Rotation standard deviation", standardDevR);
    }
}