package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.LimelightHelpers;

import java.util.Optional;

public class Vision {
    Transform3d cameraPosition;

    StructPublisher<Pose3d> pubFieldToOdometry;
    StructPublisher<Transform3d> pubRobotToCamera;

    StructSubscriber<Transform3d> subOdomToRobot;
    DoubleArraySubscriber subFieldToRobot;
    DoubleArraySubscriber subNotes;

    Pose3d fieldToOdometry3d = new Pose3d();
    Transform3d odometryCorrection = new Transform3d();
    long oldTime = 0;
    long oldDetTime = 0;


    public Vision() {

    }

    public void setOdometryPosition(Pose2d fieldToOdometry){ //Rohan
        fieldToOdometry3d = LimelightHelpers.getBotPose3d("Limelight");
        Rotation3d fieldToOdometryRotation = new Rotation3d(0,0,fieldToOdometry.getRotation().getRadians());
        if (!fieldToOdometry3d.equals(Pose3d.kZero)) {
            fieldToOdometry3d = LimelightHelpers.getBotPose3d("Limelight");
        } else{
            fieldToOdometry3d = fieldToOdometry3d;
            fieldToOdometryRotation = fieldToOdometryRotation;
        }
        pubFieldToOdometry.set(fieldToOdometry3d);
    }

    public static class TimestampedPose {
        public final double timestamp;
        public final Pose2d pose;

        public TimestampedPose(double timestamp, Pose2d pose) {
            this.timestamp = timestamp;
            this.pose = pose;
        }
    }

    public Optional<TimestampedPose> getAprilTagPose() {
        var entry = subFieldToRobot.getAtomic();
        SmartDashboard.putNumber("raw timestamp", entry.timestamp);
        if (entry.timestamp > oldTime){
            oldTime = entry.timestamp;
            Pose2d currPos = new Pose2d(entry.value[0], entry.value[1], Rotation2d.fromRadians(entry.value[2]));
            return Optional.of(new TimestampedPose(entry.serverTime/1e6, currPos));
        }
        return Optional.empty();
    }



    private Transform3d getOdometryCorrection(){//Added in timestamps to show whether subOdomToRobot has data.
        if (subOdomToRobot.getAtomic().timestamp == 0){
            return odometryCorrection;
        } else {
            return subOdomToRobot.get();
        }
    }

    protected Pose3d getRobotPosition3d() {
        return fieldToOdometry3d.transformBy(getOdometryCorrection());
    }

    public Pose2d getRobotPosition(){// Tanmaybe
        return getRobotPosition3d().toPose2d();
        //translate pose from camera to robot itself


    }

    public void setRobotPosition(Pose2d odometry, Pose2d pose){
        setOdometryPosition(odometry);
        Transform2d offSet = pose.minus(odometry);
        odometryCorrection = new Transform3d(offSet.getX(), offSet.getY(), 0, new Rotation3d(0, 0, offSet.getRotation().getRadians()));
    }



}