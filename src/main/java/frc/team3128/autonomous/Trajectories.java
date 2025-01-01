package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Robot;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.subsystems.Swerve.translationConfig;
import static frc.team3128.subsystems.Swerve.rotationConfig;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static final Swerve swerve = Swerve.getInstance();

    public static void initTrajectories() {
        Pathfinding.setPathfinder(new LocalADStar());

        AutoBuilder.configureHolonomic(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getRobotVelocity,
            swerve::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(translationConfig.kP, translationConfig.kI, translationConfig.kD),
                new PIDConstants(rotationConfig.kP, rotationConfig.kI, rotationConfig.kD),
                MAX_DRIVE_SPEED,
                DRIVE_TRACK_WIDTH / Math.sqrt(2),
                new ReplanningConfig(false, true)
            ),
            ()-> Robot.getAlliance() == Alliance.Red,
            swerve
        );
    }

    public static Command resetAuto() {
        return sequence(
            runOnce(()-> swerve.resetGyro(0)),
            runOnce(()-> swerve.resetEncoders())
        );
    }

    public static Command getPathPlannerAuto(String trajectoryName) {
        return AutoBuilder.buildAuto(trajectoryName);
    }

    public static Command goToPoint(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
            pose,
            AutoConstants.PATH_CONSTRAINTS,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }
    
}