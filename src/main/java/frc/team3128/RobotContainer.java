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

        controller.getButton(XboxButton.kLeftTrigger).onTrue(intake.setState(Intake.IntakeState.GROUND, 0));
        controller.getButton(XboxButton.kLeftBumper).onTrue(intake.setState(Intake.IntakeState.NEUTRAL, 0));

        controller.getButton(XboxButton.kRightTrigger).onTrue(shooter.setState(Shooter.ShooterState.SHOOTPRIME)).onFalse(shooter.setState(Shooter.ShooterState.SHOOTSHOOT));
        
        
        controller.getButton(XboxButton.kB).onTrue(shooter.setState(Shooter.ShooterState.KICK)).onFalse(shooter.runKickMotor(0));

        controller.getButton(XboxButton.kY).whileTrue(amper.setState(Amper.AmpState.PRIMED)).onFalse(ampFinAndDown());
        controller.getButton(XboxButton.kRightBumper).whileTrue(intake.runRollers(-1)).onFalse(intake.runRollers(0));

        controller2.getButton(XboxButton.kA).onTrue(runOnce(()-> intake.disable()).andThen(intake.pivot.reset(0)));
        controller2.getButton(XboxButton.kB).onTrue(runOnce(()-> amper.disable()).andThen(amper.reset()));
        controller2.getButton(XboxButton.kRightTrigger).onTrue(intake.pivot.runPivot(0.3));
        controller2.getButton(XboxButton.kRightBumper).onTrue(intake.pivot.runPivot(-0.3));
        


        // new Trigger(()->true).onTrue(queueNote());

        //Shooting
        new Trigger(()-> shooter.isState(Shooter.ShooterState.SHOOTSHOOT))
        .or(()-> shooter.isState(Shooter.ShooterState.AMPSHOOT))
        .onTrue(sequence(
            waitSeconds(0.25),
            hopper.runManipulator(.8)
        ))
        .onFalse(sequence(
            hopper.runManipulator(0),
            shooter.setState(Shooter.ShooterState.IDLE)
        ));
        
        //Stops shooting when all notes are gone
        new Trigger(()-> !shooter.hasObjectPresent())
        .and(()-> !hopper.hasObjectPresent())
        .debounce(0.5)
        .onTrue(sequence(
            shooter.setState(Shooter.ShooterState.IDLE)
        ));

        //Queues note to hopper
        new Trigger(()-> intake.pivot.getMeasurement() > 90)
        .and(()->!hopper.hasObjectPresent())
        .onTrue(hopper.runManipulator(HOPPER_INTAKE_POWER))
        .onFalse(hopper.runManipulator(0));

        //Stops hopper if intake is retracted and is empty
        new Trigger(()-> intake.pivot.getMeasurement() < 20)
        .and(()->!hopper.hasObjectPresent())
        .debounce(0.5)
        .onTrue(hopper.runManipulator(0));

        //Queues note to shooter
        new Trigger(()-> !shooter.hasObjectPresent())
        .and(()->hopper.hasObjectPresent())
        .and(() -> shooter.isState(Shooter.ShooterState.SHOOTSHOOT) ||( shooter.isState(Shooter.ShooterState.AMPSHOOT)))
        .onTrue(sequence(
            shooter.setState(Shooter.ShooterState.KICK),
            hopper.runManipulator(HOPPER_INTAKE_POWER)
        ))
        .onFalse(sequence(
            shooter.runKickMotor(-.1),
            waitSeconds(.1),
            shooter.runKickMotor(0)
        ));

        // new Trigger(()-> !shooter.noteInRollers())
        // .debounce(0.25)
        // .onTrue(amper.retract());
        
        // new Trigger(() -> shouldEjectNote()).onTrue(sequence(
        //     runOnce(() -> leds.setLedColor(Colors.PURPLE)),
        //     ejectNote()
        //     ));

        new Trigger(()-> hopper.hasObjectPresent())
        .and(()-> shooter.hasObjectPresent())
        .onTrue(intake.setState(Intake.IntakeState.NEUTRAL, 0)
                .andThen(intake.runRollers(0)));

        new Trigger(()-> amper.isState(Amper.AmpState.EXTENDED))
        .and(()-> shooter.hasObjectPresent())
        .onTrue(shooter.setState(Shooter.ShooterState.AMPSHOOT));

        new Trigger(()-> amper.isState(Amper.AmpState.PRIMED))
        .and(()-> shooter.hasObjectPresent())
        .onTrue(shooter.setState(Shooter.ShooterState.AMPPRIME));

        // new Trigger(()-> amper.getMeasurement() > 3)
        // .onTrue(amper.runRollers())
        // .onFalse(amper.stopRollers());

        new Trigger(()-> amper.isState(Amper.AmpState.EXTENDED) || amper.isState(Amper.AmpState.PRIMED))
        .and(()-> !shooter.hasObjectPresent())
        .debounce(0.5)
        .onTrue(amper.setState(Amper.AmpState.RETRACTED));

        new Trigger(()-> hopper.hasObjectPresent())
        .debounce(2)
        .onTrue(
            sequence(
                hopper.outtake().onlyIf(()-> !((amper.isState(Amper.AmpState.EXTENDED) || amper.isState(Amper.AmpState.PRIMED)))),
                waitSeconds(0.3)
            )
        );

    }

    

    @SuppressWarnings("unused")
    public void initCameras() {
        Camera.disableAll();
        Camera.setResources(()-> swerve.getYaw(),  (pose, time) -> swerve.addVisionMeasurement(pose, time), AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), () -> swerve.getPose());
        Camera.setThresholds(5, 0.5);
        // Camera.overrideThreshold = 30;
        // Camera.validDist = 0.5;
        // Camera.addIgnoredTags(13.0, 14.0);

        if (Robot.isReal()) {
            // final Camera camera = new Camera("FRONT_LEFT", Units.inchesToMeters(10.055), Units.inchesToMeters(9.79), Units.degreesToRadians(30), Units.degreesToRadians(-28.125), 0);
            // final Camera camera2 = new Camera("FRONT_RIGHT", Units.inchesToMeters(10.055), -Units.inchesToMeters(9.79), Units.degreesToRadians(-30), Units.degreesToRadians(-28.125), 0);
            // camera.setCamDistanceThreshold(3.5);
            // camera2.setCamDistanceThreshold(5);
        }
        // final Camera camera3 = new Camera("LEFT", Units.inchesToMeters(-3.1), Units.inchesToMeters(12.635), Units.degreesToRadians(90), Units.degreesToRadians(-10), 0);
        // final Camera camera4 = new Camera("RIGHT", Units.inchesToMeters(-3.1), Units.inchesToMeters(-12.635), Units.degreesToRadians(-90), Units.degreesToRadians(0), 0);

        // sideCams.add(camera3);
        // sideCams.add(camera4);

        // limelight = new Limelight("limelight-mason", 0, 0, 0);
    }

    public static void toggleSideCams(boolean enable) {
        for (Camera camera : sideCams) {
            if (enable) camera.enable();
            else camera.disable();
        }
    }

    public void initDashboard() {
        /*dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("time", ()-> Timer.getMatchTime());
        dashboard.addUpdate("voltage",()-> RobotController.getBatteryVoltage());
        dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
        */
    }

    public boolean isConnected() {
        for (SwerveModule module : swerve.getModules()) {
            if (module.getRunningState() != State.RUNNING) {
                Log.info("State Check", "Module " + module.moduleNumber +" failed.");
                return false;
            }
        }
        return true;
    }

    private void initRobotTest() {
        // Tester tester = Tester.getInstance();
        // tester.getTest("Robot").setTimeBetweenTests(0.5);
    }
}
