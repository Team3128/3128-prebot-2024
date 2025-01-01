package frc.team3128;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.subsystems.Swerve;
import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;

    public static Limelight limelight;

    private final Command swerveDriveCommand;

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 3;
        NAR_TalonFX.maximumRetries = 1;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        // judgePad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        controller2 = new NAR_XboxController(4);


        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX,controller::getLeftY, controller::getRightX);

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();

        configureButtonBindings();
    }   

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kA).onTrue(runOnce(()-> swerve.resetGyro(0)));
        
        controller.getButton(XboxButton.kX).onTrue(sequence(
            runOnce(()-> swerve.zeroLock(), swerve),
            swerve.characterize(1, 0.5)
        )).onFalse(runOnce(()->swerve.stop()));

        controller.getButton(XboxButton.kY).onTrue(sequence(
            runOnce(()-> swerve.oLock(), swerve),
            swerve.characterize(1, 0.5)
        )).onFalse(runOnce(()->swerve.stop()));

        controller.getButton(XboxButton.kRightStick).onTrue(runOnce(()-> swerve.snapToAngle()));

        // disables all subsystems
        controller.getButton(XboxButton.kBack).onTrue(runOnce(()-> CommandScheduler.getInstance().cancelAll()));
    }

    @SuppressWarnings("unused")
    public void initCameras() {
        Camera.disableAll();
        Camera.setResources(()-> swerve.getYaw(),  (pose, time) -> swerve.addVisionMeasurement(pose, time), AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), () -> swerve.getPose());
        Camera.setThresholds(5, 0.5);

        if (Robot.isReal()) {
            // final Camera camera = new Camera("FRONT_LEFT", Units.inchesToMeters(10.055), Units.inchesToMeters(9.79), Units.degreesToRadians(30), Units.degreesToRadians(-28.125), 0);
            // final Camera camera2 = new Camera("FRONT_RIGHT", Units.inchesToMeters(10.055), -Units.inchesToMeters(9.79), Units.degreesToRadians(-30), Units.degreesToRadians(-28.125), 0);
        }
    }

    public void initDashboard() {
        // dashboard = NarwhalDashboard.getInstance();
        // // dashboard.addUpdate("time", ()-> Timer.getMatchTime());
        // // dashboard.addUpdate("voltage",()-> RobotController.getBatteryVoltage());
        // dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        // dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        // dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
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
}
