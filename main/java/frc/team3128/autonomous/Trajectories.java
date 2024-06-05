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
import static frc.team3128.Constants.ShooterConstants.RAM_SHOT_RPM;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.commands.CmdAutoAlign;
import frc.team3128.commands.CmdSwerveDrive;

import static frc.team3128.commands.CmdManager.rampUp;

import java.util.function.DoubleSupplier;

import frc.team3128.subsystems.AmpMechanism;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
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
    private static final Climber climber = Climber.getInstance();
    private static final Shooter shooter = Shooter.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static double vx = 0, vy = 0;
    private static boolean turning = false;

    public static void initTrajectories() {
        Pathfinding.setPathfinder(new LocalADStar());

        // TODO: add commands
        NamedCommands.registerCommand("Intake", intake.intakeAuto());
        NamedCommands.registerCommand("Shoot", autoShoot(0.75));
        NamedCommands.registerCommand("RamShoot", ramShotAuto());
        NamedCommands.registerCommand("BottomShoot", autoShootPreset(ShootPosition.BOTTOM));
        NamedCommands.registerCommand("WingRamp", rampUpAuto(ShootPosition.WING));
        NamedCommands.registerCommand("Align", align());
        NamedCommands.registerCommand("AlignCCW", alignSearch(true));
        NamedCommands.registerCommand("AlignCW", alignSearch(false));
        NamedCommands.registerCommand("Outtake", outtakeAuto());
        NamedCommands.registerCommand("Retract", intake.retractAuto());
        NamedCommands.registerCommand("Neutral", neutralAuto());
        NamedCommands.registerCommand("AlignPreload", alignPreload(false));
        NamedCommands.registerCommand("Drop", shooter.shoot(MAX_RPM));
        NamedCommands.registerCommand("RamShootMax", autoShootPreset(ShootPosition.RAM));

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

    public static Command alignPreload(boolean counterClockwise) {
        return race(
            intake.intakeAuto(), 
            sequence(
                turnDegrees(false, 70).until(()-> RobotContainer.limelight.hasValidTarget()),
                repeatingSequence(
                    runOnce(()-> CmdAutoAlign.hasTimedOut = false),
                    new CmdAutoAlign(3, false),
                    run(()-> swerve.drive(
                        new Translation2d(), 
                        maxAngularVelocity / 4.0 * (counterClockwise ? 1 : -1) * (Robot.getAlliance() == Alliance.Red ? -1 : 1), 
                        false)
                    ).until(()-> RobotContainer.limelight.hasValidTarget())
                ).until(()-> intake.intakeRollers.hasObjectPresent())
            ).beforeStarting(runOnce(()->{turning = true;})).andThen(runOnce(()->{turning = false;})));
    }

    public static Command alignSearch(boolean counterClockwise) {
        return sequence(
            runOnce(()-> CmdAutoAlign.hasTimedOut = false),
            new CmdAutoAlign(3, false),
            turnDegrees(counterClockwise, 45).until(()-> RobotContainer.limelight.hasValidTarget()),
            new CmdAutoAlign(3, false)
        ).until(()-> intake.intakeRollers.hasObjectPresent())
        .beforeStarting(runOnce(()->{turning = true;})).andThen(runOnce(()->{turning = false;}));
    }

    public static Command align() {
        return sequence(
            runOnce(()-> CmdAutoAlign.hasTimedOut = false),
            new CmdAutoAlign(3, false),
            waitSeconds(1).until(()-> RobotContainer.limelight.hasValidTarget()),
            new CmdAutoAlign(3, false)
        ).until(()-> intake.intakeRollers.hasObjectPresent())
        .beforeStarting(runOnce(()->{turning = true;})).andThen(runOnce(()->{turning = false;}));
    }

    public static Command ramShotAuto() {
        return sequence(
            climber.climbTo(Climber.Setpoint.RAMSHOT),
            shooter.shoot(RAM_SHOT_RPM, RAM_SHOT_RPM),
            waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint()).withTimeout(1.5),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.35),
            shooter.shoot(MAX_RPM)
        );
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

    public static Command rampUpAuto(ShootPosition pos) {
        return either(
            sequence(
                shooter.shoot(MAX_RPM),
                rampUp(ShooterConstants.MAX_RPM, pos.getHeight()).until(()-> climber.atSetpoint()),
                intake.retractAuto()
            ),
            none(),
            ()-> true
            // ()->intake.intakeRollers.hasObjectPresent()
        );
    }

    public static Command autoShootNoTurn() {
        return sequence(
            rampUp(),
            intake.intakeRollers.outtake(),
            waitSeconds(0.25),
            intake.intakeRollers.runManipulator(0)
        );
    }

    public static Command autoShootPreset(ShootPosition position) {
        return sequence(
            parallel (
                runOnce(()->{turning = true;}),
                rampUp(MAX_RPM, position.getHeight()),
                turnInPlace().withTimeout(0.5)
            ),
            runOnce(()->{turning = false;}),
            intake.intakeRollers.outtake(),
            waitSeconds(0.25),
            intake.intakeRollers.runManipulator(0)
        );
    }
 
    public static Command autoShoot(double turnTimeout) {
        return either(
            sequence(
                either(shooter.shoot(MAX_RPM), none(), ()-> shooter.isEnabled()),
                parallel(
                    runOnce(()->{turning = true;}),
                    sequence(
                        either(sequence(waitUntil(()-> intake.intakeRollers.hasObjectPresent()).withTimeout(0.25), intake.retractAuto()), none(), ()-> intake.intakePivot.isEnabled()),
                        rampUp()
                    ),
                    turnInPlace().withTimeout(turnTimeout)
                ),
                runOnce(()->{turning = false;}),
                intake.intakeRollers.outtake(),
                waitSeconds(0.25),
                intake.intakeRollers.runManipulator(0)
            ),
            none(),
            ()-> true
        );
    }

    public static Command outtakeAuto() {
        return either(
            sequence(
                intake.intakeRollers.outtake(),
                waitSeconds(0.25),
                intake.intakeRollers.runManipulator(0)
            ),
            none(),
            ()-> true
        );
    }

    public static Command neutralAuto() {
        return sequence(
            intake.intakeRollers.runNoRequirements(0),
            // Shooter.getInstance().setShooter(0),
            Climber.getInstance().climbTo(Climber.Setpoint.RETRACTED)
        );
    }

    public static Command getPathPlannerAuto(String name) {
        return new PathPlannerAuto(name);
    }

    // TODO: make new
    public static Command resetAuto() {
        return sequence(
            Intake.getInstance().intakePivot.reset(0),
            Climber.getInstance().reset(),
            runOnce(()-> swerve.zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180)),
            AmpMechanism.getInstance().reset(-90),
            runOnce(()-> swerve.resetEncoders()),
            runOnce(()-> Intake.getInstance().isRetracting = false)
        );
        
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