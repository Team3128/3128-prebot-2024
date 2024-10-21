package frc.team3128.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.FocalAimConstants.*;
import static frc.team3128.Constants.FieldConstants.*;
import common.hardware.input.NAR_XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.Constants;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.SubsystemManager;
import frc.team3128.subsystems.Swerve;
// import frc.team3128.subsystems.Climber;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Amper amper = Amper.getInstance();
    private static Hopper hopper = Hopper.getInstance();
    //private static Climber climber = Climber.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    public static Command outtake(){
        return sequence(
            hopper.setState(Hopper.HopperState.REVERSE),
            intake.setState(Intake.IntakeState.OUTTAKE)
        );
    }

    public static Command stop(){
        return sequence(
            intake.setState(Intake.IntakeState.NEUTRAL, 0),
            amper.setState(Amper.AmpState.IDLE),
            shooter.setState(Shooter.ShooterState.IDLE),
            hopper.setState(Hopper.HopperState.IDLE)
        );
    }

    public static Command disableAll(){
        return sequence(
            intake.disable(),
            amper.disable(),
            shooter.disable(),
            hopper.disable()
        );
    }

    //better feed logic UNTESTED
    public static Command feed(double shooterPower, double angle){
        return sequence(
            parallel(
                swerve.turnInPlace(()-> allianceFlip(Rotation2d.fromDegrees(angle)).getDegrees()),
                shooter.setState(Shooter.ShooterState.SHOOT)
            ),
            waitUntil(()-> shooter.atSetpoint()),
            hopper.setState(Hopper.HopperState.FULL_FORWARD)
        );
    }

    public static Command autoShoot() {
        return sequence(
            deadline(
                SubsystemManager.getInstance().setState(SubsystemManager.RobotState.SHOOTING_RAMP, 0), 
                repeatingSequence(
                    runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                    waitSeconds(0.1)
                )
            ),
            SubsystemManager.getInstance().setState(SubsystemManager.RobotState.SHOOT_FIRST, 0), 
            runOnce(() -> CmdSwerveDrive.disableTurn())
        );
    }
}