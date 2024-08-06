package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Robot;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.*;
import static frc.team3128.Constants.FocalAimConstants.*;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Climber climber = Climber.getInstance();
    private static Amper amper = Amper.getInstance();
    private static Hopper hopper = Hopper.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    public static Command rampUpShoot() {
        return shooter.shoot(SHOOTER_RPM, SHOOTER_RPM);
    }

    public static Command ramShoot() {
        return sequence(
            rampUpShoot(),
            waitUntil(() -> shooter.atSetpoint()),
            hopper.intake(),
            waitSeconds(0.35),
            neutral(false)
        );
    }

    public static Command autoShoot() {
        return deadline(
            ramShoot(),
            repeatingSequence(
                runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                waitSeconds(0.1)
            )
        ).andThen(runOnce(() -> CmdSwerveDrive.disableTurn()));
    }

    public static Command rampUpAmp() {
        return sequence(
            shooter.shoot(ShooterConstants.AMP_RPM),
            amper.extend()
        );
    }

    public static Command amp() {
        return sequence(
            rampUpAmp(),
            waitUntil(() -> shooter.atSetpoint() && amper.atSetpoint()),
            hopper.intake(),
            waitSeconds(0.35),
            neutral(false)
        );
    }

    // TODO: this
    // public static Command autoAmp() {
        
    // }

    public static Command neutral(boolean shouldStall) {
        return sequence(
            intake.stopRollers(),
            shooter.setShooter(0),
            amper.retract(),
            hopper.runManipulator(0)
        );
    }
}