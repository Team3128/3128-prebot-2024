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
import frc.team3128.Robot;
import frc.team3128.RobotContainer;

import frc.team3128.subsystems.SubsystemManager;
import frc.team3128.subsystems.Swerve;
// import frc.team3128.subsystems.Climber;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();

    //private static Climber climber = Climber.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    public static Command disableAll(){
        return sequence(
            
        );
    }
}