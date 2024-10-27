// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import common.core.misc.NAR_Robot;
import common.hardware.camera.Camera;
import common.utility.Log;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.commands.CmdManager;
import frc.team3128.subsystems.SubsystemManager;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.SubsystemManager.RobotState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends NAR_Robot {
    private Timer m_gcTimer = new Timer();

    private boolean hasInitialized = false;
    private int notePlateuCount = 0;

    private static Alliance alliance;
    private static boolean refreshCachedAlliance;

    public static Alliance getAlliance() {
        if (alliance == null || refreshCachedAlliance) {
            Optional<Alliance> DSalliance = DriverStation.getAlliance();
            if (DSalliance.isPresent()) alliance = DSalliance.get();
            refreshCachedAlliance = false;
        }
        return alliance;
    }

    public static Robot instance;

    public static RobotContainer m_robotContainer = new RobotContainer();
    public static AutoPrograms autoPrograms;

    public static synchronized Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    @Override
    public void robotInit(){
        m_gcTimer.restart();
        refreshCachedAlliance = true;
        autoPrograms = new AutoPrograms();
        m_robotContainer.initDashboard();
        LiveWindow.disableAllTelemetry();
        // runOnce(()-> Swerve.getInstance().zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180));
        // Swerve.getInstance().resetOdometry((new Pose2d(new Translation2d(1.45, 4.1), Rotation2d.fromDegrees(180)))); //1.45, 4.1
        // Alliance allianceTemp = getAlliance();
        // if (allianceTemp == null) {
        //     Log.info("Alliance", "Did not have correct color");
        // }
        // else {
        //     Log.info("Alliance", "We are alliance " + allianceTemp);
        // }
        // Log.info("Gyro Angle", "" + Swerve.getInstance().getYaw());
    }

    @Override
    public void driverStationConnected() {
        refreshCachedAlliance = true;
        Log.info("State", "DS Connected");
        Log.info("Alliance", getAlliance().toString());
        if (getAlliance() == Alliance.Red) {
            Camera.addIgnoredTags(3, 4, 5, 11, 12);
        } else {
            Camera.addIgnoredTags(6, 7, 8, 15, 16);
        }
        if (!NAR_Robot.logWithAdvantageKit) return;
        if(DriverStation.getMatchType() != MatchType.None){
            addReceiver(true, LoggingState.FULLMATCH);
 
        }else{
            addReceiver(true, LoggingState.SESSION);
        }

        Logger.start();
    }

    @Override
    public void robotPeriodic(){
        Camera.updateAll();
        if(m_gcTimer.advanceIfElapsed(5)) {
            System.gc();
        }
    }

    @Override
    public void autonomousInit() {
        refreshCachedAlliance = true;
        Camera.enableAll();
        Camera.overrideThreshold = 0;
        Camera.validDist = 30;
        Command m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        refreshCachedAlliance = true;
        Camera.overrideThreshold = 30;
        Camera.validDist = 0.5;
        Camera.enableAll();
        SubsystemManager.getInstance().setState(RobotState.FULL_IDLE, 0);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {
        refreshCachedAlliance = true;
    }

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        refreshCachedAlliance = true;
        Swerve.getInstance().setBrakeMode(true);
        CommandScheduler.getInstance().cancelAll();

        CmdManager.disableAll().schedule();
        sequence(
            waitSeconds(3.0).ignoringDisable(true),
            runOnce(()->Swerve.getInstance().setBrakeMode(false)).ignoringDisable(true)
        ).schedule();

        if (hasInitialized) {
        }
        hasInitialized = true;
    }

    @Override
    public void disabledExit() {
        Swerve.getInstance().setBrakeMode(true
        );
    }
    
    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
