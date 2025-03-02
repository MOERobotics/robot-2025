package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import java.util.ArrayList;

public interface SwerveDriveControl extends Subsystem {
    @AutoLog
    class SwerveDriveInputs {
        public double currentRotationRadians;
        public Pose2d pose;
        public SwerveModuleState[] moduleStates;
        public SwerveModuleState[] driveDesiredStates;
        public SwerveModulePosition[] modulePositions;
        public SwerveModuleInputsAutoLogged swerveModuleFL;
        public SwerveModuleInputsAutoLogged swerveModuleFR;
        public SwerveModuleInputsAutoLogged swerveModuleBR;
        public SwerveModuleInputsAutoLogged swerveModuleBL;
        public ChassisSpeeds robotRelativeSpeeds;
        public Pose2d Pose2dFromLL = new Pose2d();
        public boolean canClimb;
    }

    default Pose2d getPose() {
        return this.getSensors().pose;
    }

    SwerveDriveInputsAutoLogged getSensors();

    default void resetPose(Pose2d newPose) {
        this.getOdometry().resetPose(newPose);
    }

    void drive(ChassisSpeeds speeds);

    default void drive(double xSpeed, double ySpeed, double rotation, boolean robotCentric) {
        drive(new ChassisSpeeds(xSpeed, ySpeed, rotation));
    }

    default void drive(double xSpeed, double ySpeed, double rotation) {
        drive(xSpeed, ySpeed, rotation, false);
    }

    default ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getSensors().robotRelativeSpeeds;
    }

    SwerveDriveKinematics getKinematics();

    SwerveDriveOdometry getOdometry();

    default boolean canClimb() {
        return this.getSensors().canClimb;
    }

    default Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints) {
        return generateTrajectory(start, end, internalPoints, 0,0);
    }


    default Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints, double startVelocityMetersPerSecond, double endVelocityMetersPerSecond) {
        return Commands.none();
    }

    void driveSingleModule (int index, double xSpeed, double ySpeed, double rotation);


}
