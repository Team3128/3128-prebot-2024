package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Robot;

import java.util.function.DoubleSupplier;

import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Intake.IntakeState;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.SubsystemManager;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static final Swerve swerve = Swerve.getInstance();
    private static double vx = 0, vy = 0;
    private static boolean turning = false;

    public static void initTrajectories() {
        Pathfinding.setPathfinder(new LocalADStar());

        // TODO: add commands
        NamedCommands.registerCommand("Shoot", runOnce(() -> {}));//shoot());
        NamedCommands.registerCommand("Intake", runOnce(() -> {}));//Intake.getInstance().setState(IntakeState.GROUND));
        NamedCommands.registerCommand("Neutral", runOnce(() -> {}));//SubsystemManager.getInstance().setState(SubsystemManager.RobotState.FULL_IDLE, 0));

        AutoBuilder.configureHolonomic(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getRobotVelocity,
            swerve::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
                new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD),
                MAX_ATTAINABLE_DRIVE_SPEED,
                DRIVE_TRACK_WIDTH,
                new ReplanningConfig(false, true)
            ),
            ()-> Robot.getAlliance() == Alliance.Red,
            swerve
        );
    }

    public static Command shoot() {
        return sequence(
            SubsystemManager.getInstance().setState(SubsystemManager.RobotState.SHOOTING, 0),
            waitSeconds(1)
            // waitUntil(()-> Hopper.hasNoObjects()),
            // SubsystemManager.getInstance().setState(SubsystemManager.RobotState.FULL_IDLE, 0)
        ).withTimeout(1);
    }

    public static Command resetAuto() {
        return sequence(
            runOnce(()-> swerve.zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180)),
            runOnce(()-> swerve.resetEncoders()),
            Amper.getInstance().reset(),
            Intake.getInstance().reset()
        );
    }

    public static Command turnDegrees(boolean counterClockwise, double angle) {
        DoubleSupplier setpoint = ()-> swerve.getYaw() + angle * (counterClockwise ? 1 : -1) * (Robot.getAlliance() == Alliance.Red ? -1 : 1);
        return swerve.turnInPlace(setpoint);
    }

    public static Command turnInPlace() {
        DoubleSupplier setpoint = ()-> swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue);
        return swerve.turnInPlace(setpoint);
    }

    public static Command getPathPlannerAuto(String name) {
        return new PathPlannerAuto(name);
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