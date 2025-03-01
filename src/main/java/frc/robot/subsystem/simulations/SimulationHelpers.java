package frc.robot.subsystem.simulations;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.*;

import static edu.wpi.first.units.Units.RPM;

public class SimulationHelpers {
    private static SwerveDriveSim swerveDriveSim;
    private static SwerveModuleSim swerveModuleSimFL, swerveModuleSimFR, swerveModuleSimBR, swerveModuleSimBL;
    private static RobotElevatorSim elevatorSim;
    private static AlgaeCollectorSim algaeCollectorSim;
    private static CoralCollectorSim coralCollectorSim;
    private static ClimberSim climberMidSim, climberRearSim;

    public static AngularVelocity decelerate(AngularVelocity velocity, double decelerationCoef) {
        if (velocity.isNear(RPM.zero(), 0.01)) {
            return RPM.zero();
        } else {
            return velocity.abs(RPM) > decelerationCoef ? velocity.minus(RPM.of(Math.copySign(decelerationCoef, velocity.in(RPM)))) : RPM.zero();
        }
    }

    public static AngularVelocity getSystemVelocity(AngularVelocity motorVelocity, double gearing) {
        return motorVelocity.div(gearing);
    }

    public static AngularVelocity getMotorVelocity(AngularVelocity systemVelocity, double gearing) {
        return systemVelocity.times(gearing);
    }

    public static void setUpSim(RobotContainer robot) {

        if (!(robot.getSwerveDrive() instanceof SwerveDrive swerveDrive)
            || !(robot.getElevator() instanceof SubMOErineElevator elevator)
            || !(robot.getAlgaeCollector() instanceof AlgaeCollector)
            || !(robot.getCoralHead() instanceof CoralHead)
            || !(robot.getClimberRear() instanceof SubMOErineClimber)
            || !(robot.getClimberMid() instanceof SubMOErineClimber)) throw new InstantiationError("Robot Container should be SubMOErine but  is actually"+robot.getClass().getSimpleName());
        swerveDriveSim = new SwerveDriveSim(swerveDrive);
        swerveModuleSimFL = new SwerveModuleSim(swerveDrive.swerveModuleFL.moduleOffset, swerveDrive.swerveModuleFL.driveMotor, swerveDrive.swerveModuleFL.pivotMotor, swerveDrive.swerveModuleFL.pivotEncoder);
        swerveModuleSimFR = new SwerveModuleSim(swerveDrive.swerveModuleFR.moduleOffset, swerveDrive.swerveModuleFR.driveMotor, swerveDrive.swerveModuleFR.pivotMotor, swerveDrive.swerveModuleFR.pivotEncoder);
        swerveModuleSimBR = new SwerveModuleSim(swerveDrive.swerveModuleBR.moduleOffset, swerveDrive.swerveModuleBR.driveMotor, swerveDrive.swerveModuleBR.pivotMotor, swerveDrive.swerveModuleBR.pivotEncoder);
        swerveModuleSimBL = new SwerveModuleSim(swerveDrive.swerveModuleBL.moduleOffset, swerveDrive.swerveModuleBL.driveMotor, swerveDrive.swerveModuleBL.pivotMotor, swerveDrive.swerveModuleBL.pivotEncoder);
        elevatorSim = new RobotElevatorSim(elevator);
        algaeCollectorSim = new AlgaeCollectorSim((AlgaeCollector) robot.getAlgaeCollector());
        coralCollectorSim = new CoralCollectorSim((CoralHead) robot.getCoralHead());
        climberMidSim = new ClimberSim((SubMOErineClimber) robot.getClimberMid());
        climberRearSim = new ClimberSim((SubMOErineClimber) robot.getClimberRear());
    }

    public static void updateSimState() {
        swerveDriveSim.updateSimState();
        swerveModuleSimFL.updateSimState();
        swerveModuleSimFR.updateSimState();
        swerveModuleSimBR.updateSimState();
        swerveModuleSimBL.updateSimState();
        elevatorSim.updateSimState();
        algaeCollectorSim.updateSimState();
        coralCollectorSim.updateSimState();
        climberMidSim.updateSimState();
        climberRearSim.updateSimState();
    }
}
