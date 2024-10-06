package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.commands.NAR_PIDCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Robot;
import frc.team3128.commands.CmdManager;
import frc.team3128.commands.CmdSwerveDrive;

import java.util.function.DoubleSupplier;

import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Intake;

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
        NamedCommands.registerCommand("ramShoot", CmdManager.ramShoot(true));
        NamedCommands.registerCommand("ramShootNoStop", CmdManager.ramShoot(false));
        NamedCommands.registerCommand("intakeAndStop", Intake.getInstance().setState(Intake.IntakeState.INTAKE));

        AutoBuilder.configureHolonomic(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getRobotVelocity,
            Trajectories::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(translationKP, translationKI, translationKD),
                new PIDConstants(rotationKP, rotationKI, rotationKD),
                maxAttainableSpeed,
                trackWidth,
                new ReplanningConfig(false, true)
            ),
            ()-> Robot.getAlliance() == Alliance.Red,
            swerve
        );
    }

    public static Command resetAuto() {
        return sequence(
            runOnce(()-> swerve.zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180)),
            runOnce(()-> swerve.resetEncoders())
        );
    }

    public static void drive(ChassisSpeeds velocity) {
        if (!turning) swerve.drive(velocity);
        else {
            vx = velocity.vxMetersPerSecond;
            vy = velocity.vyMetersPerSecond;
        }
    }

    public Command turnInPlace(DoubleSupplier setpoint, double timeout) {
        TURN_CONTROLLER.enableContinuousInput(-180, 180);
        return new NAR_PIDCommand(
            TURN_CONTROLLER, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
            },
            timeout,
            Swerve.getInstance()
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command getPathPlannerAuto(String name) {
        return new PathPlannerAuto(name);
    }

    public static Command goToPoint(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
            pose,
            AutoConstants.constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }
    
}