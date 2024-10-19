package frc.team3128;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.team3128.commands.CmdManager.disableAll;
import static frc.team3128.commands.CmdManager.outtake;
import static frc.team3128.commands.CmdManager.stop;

import java.util.ArrayList;

import common.core.swerve.SwerveModule;
import common.hardware.camera.Camera;
import common.hardware.input.NAR_ButtonBoard;
import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.limelight.Limelight;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Amper.AmpState;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Hopper.HopperState;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Intake.IntakeState;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Shooter.ShooterState;
import frc.team3128.subsystems.Swerve;

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

        amper.initTriggers();
        hopper.initTriggers();
        intake.initTriggers();
        shooter.initTriggers();

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

        // zero gyro
        controller.getButton(XboxButton.kStart).onTrue(runOnce(()-> swerve.zeroGyro(0)));

        // intake ground and then neutral
        controller.getButton(XboxButton.kLeftTrigger).onTrue(intake.setState(Intake.IntakeState.GROUND)).onFalse(intake.setState(Intake.IntakeState.NEUTRAL));

        // intake neutral
        controller.getButton(XboxButton.kLeftBumper).onTrue(intake.setState(Intake.IntakeState.NEUTRAL));

        // ramp shooter and then run hopper
        // shooter and hopper will stop if no notes
        controller.getButton(XboxButton.kRightTrigger).onTrue(shooter.setState(Shooter.ShooterState.SHOOT)).onFalse(hopper.setState(Hopper.HopperState.SHOOT));

        // shooter ramp amp
        controller.getButton(XboxButton.kA).onTrue(shooter.setState(Shooter.ShooterState.AMP));

        // amper primed and then extended
        controller.getButton(XboxButton.kY).onTrue(amper.setState(Amper.AmpState.PRIMED)).onFalse(amper.setState(Amper.AmpState.EXTENDED));

        // manual hopper button
        controller.getButton(XboxButton.kB).onTrue(hopper.setState(HopperState.INTAKE)).onFalse(hopper.disable());

        // runs everything in reverse at max power and then go to neutral
        controller.getButton(XboxButton.kRightBumper).whileTrue(outtake()).onFalse(stop());

        // disables all subsystems
        controller.getButton(XboxButton.kBack).onTrue(disableAll());

        controller2.getButton(XboxButton.kA).onTrue(runOnce(()-> intake.disable()).andThen(intake.pivot.reset(0)));
        controller2.getButton(XboxButton.kB).onTrue(runOnce(()-> amper.disable()).andThen(amper.reset()));
        controller2.getButton(XboxButton.kRightTrigger).whileTrue(intake.pivot.runPivot(0.3)).onFalse(intake.pivot.runPivot(0));
        controller2.getButton(XboxButton.kRightBumper).whileTrue(intake.pivot.runPivot(-0.3)).onFalse(intake.pivot.runPivot(0));
        controller2.getButton(XboxButton.kLeftTrigger).whileTrue(amper.elevator.runElevator(0.3)).onFalse(amper.elevator.runElevator(0));
        controller2.getButton(XboxButton.kLeftBumper).whileTrue(amper.elevator.runElevator(-0.3)).onFalse(amper.elevator.runElevator(0));
        controller2.getButton(XboxButton.kY).onTrue(intake.rollers.runShooter(0.65)).onFalse(intake.rollers.runShooter(0));
        controller2.getButton(XboxButton.kX).onTrue(intake.rollers.runShooter(-0.65)).onFalse(intake.rollers.runShooter(0));

        //HOPPER TRIGGERS (LOWEST PRIO)
        new Trigger(()-> hopper.isState(HopperState.SHOOT))
        .and(()-> Hopper.hasNoObjects())
        .debounce(0.25)
        .onTrue(hopper.setState(HopperState.IDLE));

        new Trigger(()-> intake.isState(Intake.IntakeState.GROUND))
        .onTrue(hopper.setState(HopperState.INTAKE));

        new Trigger(()-> hopper.isState(HopperState.INTAKE))
        .and(()-> Hopper.hasTwoObjects())
        .onTrue(hopper.setState(HopperState.IDLE));

        new Trigger(()-> intake.isState(Intake.IntakeState.NEUTRAL))
        .and(()-> !hopper.isState(HopperState.SHOOT))
        .onTrue(hopper.setState(HopperState.IDLE));

        new Trigger(()-> amper.isState(Amper.AmpState.EXTENDED))
        .onTrue(hopper.setState(HopperState.SHOOT));

        //INTAKE TRIGGERS
        new Trigger(()-> intake.isState(IntakeState.GROUND))
        .and(()-> Hopper.hasTwoObjects())
        .onTrue(intake.setState(IntakeState.NEUTRAL));

        //AMPER TRIGGERS
        new Trigger(()-> amper.isState(AmpState.EXTENDED))
        .and(()-> Hopper.hasNoObjects())
        .debounce(0.5)
        .onTrue(amper.setState(AmpState.IDLE));

        //SHOOTER TRIGGERS (HIGHEST PRIO)
        // stop if no notes
        new Trigger(()-> Hopper.hasNoObjects())
        .debounce(0.25)
        .onTrue(shooter.setState(ShooterState.IDLE));
        
        //follow amper
        new Trigger(()-> Amper.getInstance().isState(Amper.AmpState.PRIMED))
        .onTrue(shooter.setState(ShooterState.AMP));

        new Trigger(()-> Amper.getInstance().isState(Amper.AmpState.EXTENDED))
        .onTrue(shooter.setState(ShooterState.AMP));

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
