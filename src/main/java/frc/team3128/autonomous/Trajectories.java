package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.commands.NAR_PIDCommand;
import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.controllers.Controller.Type;
import common.utility.shuffleboard.NAR_Shuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.ShooterConstants.MAX_RPM;
import static frc.team3128.Constants.ShooterConstants.SHOOTER_RPM;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.commands.CmdSwerveDrive;

import java.util.function.DoubleSupplier;

import frc.team3128.subsystems.Swerve;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    //USED FOR HARDCODED SHOTS BITCHES
    public enum ShootPosition {
        // find values
        WING(5.09),      //Change this for top 
        BOTTOM(7.5),    //Change this for bottom auto
        RAM(24.5);

        private final double height;
        ShootPosition(double height) {
            this.height = height;
        }
        public double getHeight() {
            return height;
        }
    }

    private static final Swerve swerve = Swerve.getInstance();
    private static double vx = 0, vy = 0;
    private static boolean turning = false;

    public static void initTrajectories() {
        Pathfinding.setPathfinder(new LocalADStar());

        // TODO: add commands
        // NamedCommands.registerCommand("Intake", intake.intakeAuto());
        // NamedCommands.registerCommand("Shoot", autoShoot(0.75));
        // NamedCommands.registerCommand("RamShoot", ramShotAuto());
        // NamedCommands.registerCommand("BottomShoot", autoShootPreset(ShootPosition.BOTTOM));
        // NamedCommands.registerCommand("WingRamp", rampUpAuto(ShootPosition.WING));
        // NamedCommands.registerCommand("Align", align());
        // NamedCommands.registerCommand("AlignCCW", alignSearch(true));
        // NamedCommands.registerCommand("AlignCW", alignSearch(false));
        // NamedCommands.registerCommand("Outtake", outtakeAuto());
        // NamedCommands.registerCommand("Retract", intake.retractAuto());
        // NamedCommands.registerCommand("Neutral", neutralAuto());
        // NamedCommands.registerCommand("AlignPreload", alignPreload(false));
        // NamedCommands.registerCommand("Drop", shooter.shoot(MAX_RPM));
        // NamedCommands.registerCommand("RamShootMax", autoShootPreset(ShootPosition.RAM));

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

    public static Command turnDegrees(boolean counterClockwise, double angle) {
        DoubleSupplier setpoint = ()-> swerve.getYaw() + angle * (counterClockwise ? 1 : -1) * (Robot.getAlliance() == Alliance.Red ? -1 : 1);
        Controller controller = new Controller(new PIDFFConfig(5, 0, 0, turnkS, 0, 0), Type.POSITION);
        controller.enableContinuousInput(-180, 180);
        controller.setTolerance(1);
        return new NAR_PIDCommand(
            controller, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                Swerve.getInstance().drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
                NAR_Shuffleboard.addData("HElp", "help", output, 0, 0);
            }
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command turnInPlace() {
        DoubleSupplier setpoint = ()-> swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue);
        Controller controller = new Controller(new PIDFFConfig(5, 0, 0, turnkS, 0, 0), Type.POSITION);
        controller.enableContinuousInput(-180, 180);
        controller.setTolerance(1);
        return new NAR_PIDCommand(
            controller, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                Swerve.getInstance().drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
                NAR_Shuffleboard.addData("HElp", "help", output, 0, 0);
            }
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