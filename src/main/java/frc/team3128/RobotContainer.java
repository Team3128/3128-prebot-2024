package frc.team3128;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.team3128.Constants.AmperConstants.AMP_DURATION;
import static frc.team3128.Constants.ShooterConstants.SHOOT_DURATION;
import static frc.team3128.commands.CmdManager.disableAll;

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
import common.utility.sysid.CmdSysId;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.SubsystemManager;
import frc.team3128.subsystems.SubsystemManager.RobotState;
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
    private SubsystemManager robot;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;

    public static Limelight limelight;

    private static ArrayList<Camera> sideCams = new ArrayList<Camera>();

    private final CmdSwerveDrive swerveDriveCommand;

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
        robot = SubsystemManager.getInstance();

        swerveDriveCommand = new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true);

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        initRobotTest();
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();

        configureButtonBindings();


        // NAR_Shuffleboard.addData("Limelight", "ValidTarget", ()-> limelight.hasValidTarget(), 0, 0);
        // NAR_Shuffleboard.addData("Limelight", "TX", ()-> limelight.getValue(LimelightKey.HORIZONTAL_OFFSET), 0, 1);
        // NAR_Shuffleboard.addData("Hopper", "Kicker Note", ()-> Hopper.kickerHasObjectPresent(), 0, 0);
        // NAR_Shuffleboard.addData("Hopper", "Hopper Note", ()-> Hopper.hopperHasObjectPresent(), 1, 0);
        // NAR_Shuffleboard.addData("Hopper", "Has Two Notes", ()-> Hopper.hasTwoObjects(), 2, 0);
        // NAR_Shuffleboard.addData("Hopper", "Has No Notes", ()-> Hopper.hasNoObjects(), 3, 0);
    }   

    private void configureButtonBindings() {
        // controller.getButton(XboxButton.kX).onTrue(Commands.runOnce(()-> swerve.zeroGyro(0)));
        controller.getButton(XboxButton.kX).onTrue(new CmdSysId("Swerve", (Double voltage) -> swerve.setVoltage(voltage), ()->swerve.getModules()[0].getDriveMotor().getVelocity(), 
        ()->swerve.getModules()[0].getDriveMotor().getPosition(), 50,true, swerve));

        controller.getButton(XboxButton.kRightStick).onTrue(runOnce(()-> swerveDriveCommand.setTurnSetpoint()));
        controller.getUpPOVButton().onTrue(runOnce(()->swerve.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 180 : 0)));
        controller.getDownPOVButton().onTrue(runOnce(()-> swerve.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 0 : 180)));
        controller.getRightPOVButton().onTrue(runOnce(()-> swerve.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 90 : 270)));
        controller.getLeftPOVButton().onTrue(runOnce(()-> swerve.setTurnSetpoint(Robot.getAlliance() == Alliance.Red ? 270 : 90)));

        // zero gyro
        controller.getButton(XboxButton.kStart).onTrue(runOnce(()-> swerve.zeroGyro(0)));

        // intake ground and then neutral
        controller.getButton(XboxButton.kLeftTrigger).onTrue(robot.setState(RobotState.INTAKING, 0).onlyIf(()->!Hopper.hasTwoObjects())).onFalse(robot.setState(RobotState.IDLE, 0));

        // intake neutral
        controller.getButton(XboxButton.kLeftBumper).onTrue(robot.setState(RobotState.IDLE, 0));

        // ramp shooter and then run hopper
        // shooter and hopper will stop if no notes
        controller.getButton(XboxButton.kRightTrigger).onTrue(robot.setState(RobotState.SHOOTING_READY, 0)).onFalse(robot.setState(RobotState.SHOOTING, 0).andThen(robot.setState(RobotState.IDLE, SHOOT_DURATION)));

        // shooter ramp amp
        // controller.getButton(XboxButton.kA).onTrue(shooter.setState(Shooter.ShooterState.AMP));

        // amper primed and then extended
        controller.getButton(XboxButton.kY).onTrue(robot.setState(RobotState.AMPING_READY, 0)).onFalse(robot.setState(RobotState.AMPING, 0).andThen(robot.setState(RobotState.IDLE, AMP_DURATION)));

        // manual hopper button
        // controller.getButton(XboxButton.kB).onTrue(hopper.setState(HopperState.INTAKE)).onFalse(hopper.disable());

        // runs everything in reverse at max power and then go to neutral
        controller.getButton(XboxButton.kRightBumper).whileTrue(robot.setState(RobotState.OUTTAKING, 0)).onFalse(robot.setState(RobotState.IDLE, 0));

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

        // // auto eject
        // new Trigger(()-> Hopper.hopperHasObjectPresent())
        // .debounce(2.5)
        // .whileTrue(hopper.setState(HopperState.HOPPER_BACKWARD));

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

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        // dashboard.addUpdate("time", ()-> Timer.getMatchTime());
        // dashboard.addUpdate("voltage",()-> RobotController.getBatteryVoltage());
        dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
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
