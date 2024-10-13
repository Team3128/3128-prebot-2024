package frc.team3128;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.*;
import static frc.team3128.Constants.IntakeConstants.*;
import static frc.team3128.Constants.AmperConstants.*;
import static frc.team3128.Constants.HopperConstants.*;
import static frc.team3128.Constants.Flags.*;
import static frc.team3128.commands.CmdManager.*;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.team3128.Constants.IntakeConstants;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.commands.CmdSwerveDrive;
import common.core.swerve.SwerveModule;
import common.hardware.camera.Camera;
// import common.hardware.camera.OffseasonAprilTags;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.limelight.Limelight;
import common.hardware.limelight.LimelightKey;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.CmdSysId;
import common.utility.tester.Tester;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
// import common.utility.tester.Tester.UnitTest;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;
import java.util.ArrayList;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Amper amper;
    private Hopper hopper;
    private Intake intake;
    private Shooter shooter;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;

    public static Limelight limelight;

    private static ArrayList<Camera> sideCams = new ArrayList<Camera>();

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 3;
        NAR_TalonFX.maximumRetries = 1;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        // judgePad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        controller2 = new NAR_XboxController(4);

        swerve = Swerve.getInstance();
        amper = Amper.getInstance();
        hopper = Hopper.getInstance();
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));

        initRobotTest();
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();

        configureButtonBindings();


        NAR_Shuffleboard.addData("Sensors", "Hopper", ()-> hopper.hasObjectPresent(), 0, 0);
        NAR_Shuffleboard.addData("Sensor", "Shooter", () -> shooter.noteInRollers(), 0, 1);
        // NAR_Shuffleboard.addData("Limelight", "ValidTarget", ()-> limelight.hasValidTarget(), 0, 0);
        // NAR_Shuffleboard.addData("Limelight", "TX", ()-> limelight.getValue(LimelightKey.HORIZONTAL_OFFSET), 0, 1);
    }   

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kX).onTrue(Commands.runOnce(()-> swerve.zeroGyro(0)));

        controller.getButton(XboxButton.kRightStick).onTrue(runOnce(()-> CmdSwerveDrive.setTurnSetpoint()));
        controller.getUpPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 180 : 0);
        }));
        controller.getDownPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 0 : 180);
        }));

        controller.getRightPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 90 : 270);
        }));

        controller.getLeftPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 270 : 90);
        }));

        controller.getButton(XboxButton.kStart).onTrue(runOnce(()-> swerve.zeroGyro(0)));

        controller.getButton(XboxButton.kLeftTrigger).onTrue(intake(Intake.Setpoint.GROUND));
        controller.getButton(XboxButton.kLeftBumper).onTrue(retractIntake());

        controller.getButton(XboxButton.kRightTrigger).onTrue(shooter.rampUpShooter()).onFalse(shooter.setShooting(true));

        controller.getButton(XboxButton.kA).onTrue(shooter.runShooter(0.8));
        controller.getButton(XboxButton.kY).onTrue(shooter.runShooter(0));
        // controller.getButton(XboxButton.kB).onTrue(shooter.runKickMotor(KICK_SHOOTING_POWER)).onFalse(shooter.runKickMotor(0));

        controller.getButton(XboxButton.kY).whileTrue(amper.partExtend()).onFalse(ampFinAndDown());
        controller.getButton(XboxButton.kRightBumper).whileTrue(outtake()).onFalse(stop());

        //feed
        controller.getButton(XboxButton.kB).onTrue(feed(1, EDGE_FEED_ANGLE));

        controller2.getButton(XboxButton.kA).onTrue(runOnce(()-> intake.disable()).andThen(intake.reset(0)));
        controller2.getButton(XboxButton.kB).onTrue(runOnce(()-> amper.disable()).andThen(amper.reset(0)));
        controller2.getButton(XboxButton.kRightTrigger).onTrue(intake.runPivot(0.3));
        controller2.getButton(XboxButton.kRightBumper).onTrue(intake.runPivot(-0.3));
        controller2.getButton(XboxButton.kLeftTrigger).onTrue(amper.runElevator(0.3));
        controller2.getButton(XboxButton.kLeftBumper).onTrue(amper.runElevator(-0.3));

        // eject
        new Trigger(()-> hopper.hasObjectPresent())
        .and(()-> shooter.noteInRollers())
        .debounce(2)
        .onTrue(
            sequence(
                hopper.outtake().onlyIf(()-> !(SHOOTER_MOTOR.getAppliedOutput() > 0.1)),
                waitSeconds(0.35)
            )
        );

        // new Trigger(()->true).onTrue(queueNote());

        new Trigger(shooterHasNote).onTrue(vibrateController());

        // //Shooting
        // new Trigger(()-> shooter.getShooting())
        // .onTrue(sequence(
        //     shoTTT5555
    }
}