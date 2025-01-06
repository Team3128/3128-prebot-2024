package frc.team3128.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import common.utility.Log;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Robot;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.subsystems.Swerve.translationConfig;

import java.util.ArrayList;

import static frc.team3128.subsystems.Swerve.rotationConfig;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static final Swerve swerve = Swerve.getInstance();

    public static void initTrajectories() {
        Pathfinding.setPathfinder(new LocalADStar());

        RobotConfig robotConfig;

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            robotConfig = new RobotConfig(
                ROBOT_MASS,
                ROBOT_MOI, 
                new ModuleConfig(
                    DRIVE_WHEEL_DIAMETER / 2, 
                    MAX_DRIVE_SPEED, 
                    WHEEL_COF, 
                    DCMotor.getKrakenX60(1),
                    DRIVE_MOTOR_GEAR_RATIO, 
                    (double) DRIVE_MOTOR_CURRENT_LIMIT, 
                    1
                ),
                Swerve.moduleOffsets
            );
        }

        AutoBuilder.configure(
            swerve::getPose, 
            swerve::resetOdometry, 
            swerve::getRobotVelocity, 
            (velocity, feedforwards)-> swerve.drive(velocity), 
            new PPHolonomicDriveController(
                new PIDConstants(translationConfig.kP, translationConfig.kI, translationConfig.kD),
                new PIDConstants(rotationConfig.kP, rotationConfig.kI, rotationConfig.kD)
            ),
            robotConfig,
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
}