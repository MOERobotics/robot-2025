package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    public SwerveModule frontRightModule;

    public SwerveModule frontLeftModule;

    public SwerveModule backRightModule;

    public SwerveModule backLeftModule;

    public SwerveDriveKinematics swerveDriveKinematics;




    public static class Inputs {
        double initialRotation;
        double currentRotation;
    }

    public SwerveDrive(SwerveModule frontLeftModule,
                       SwerveModule backLeftModule,
                       SwerveModule frontRightModule,
                       SwerveModule backRightModule){

        this.frontLeftModule=frontLeftModule;
        this.backLeftModule=backLeftModule;
        this.frontRightModule=frontRightModule;
        this.backRightModule=frontLeftModule;
        swerveDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(frontLeftModule.xLocation,frontLeftModule.yLocation),                new Translation2d(frontLeftModule.xLocation,frontLeftModule.yLocation),
                new Translation2d(frontRightModule.xLocation,frontRightModule.yLocation),
                new Translation2d(backRightModule.xLocation,backRightModule.yLocation),
                new Translation2d(backLeftModule.xLocation,backLeftModule.yLocation)

                );

    }

    public void drive( double xpower, double ypower, double turnspd){

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xpower,ypower,turnspd);

         SwerveModuleState [] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

         frontLeftModule.setDriveMotor(swerveModuleStates[0].speedMetersPerSecond);
         frontRightModule.setDriveMotor(swerveModuleStates[1].speedMetersPerSecond);
         backRightModule.setDriveMotor(swerveModuleStates[2].speedMetersPerSecond);
         backLeftModule.setDriveMotor(swerveModuleStates[3].speedMetersPerSecond);

        frontLeftModule.setPivotMotor(swerveModuleStates[0].angle.getMeasure());
        frontRightModule.setPivotMotor(swerveModuleStates[1].angle.getMeasure());
        backRightModule.setPivotMotor(swerveModuleStates[2].angle.getMeasure());
        backLeftModule.setPivotMotor(swerveModuleStates[3].angle.getMeasure());




    }


}
