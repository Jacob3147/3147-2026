// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
        m_robotContainer.teleopInit = false;
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
        m_robotContainer.teleopInit = false;

  }

  String auto_selected = "";
  String auto_selected_prev = "";
  @Override
  public void disabledPeriodic() 
  {
    SmartDashboard.putBoolean("hub active", true);
        m_robotContainer.teleopInit = false;


    //display our auto on dashboard
    try {
      auto_selected = m_robotContainer.getAutonomousCommand().getName();
      if(auto_selected != auto_selected_prev)
      {
        Pose2d starting_pose;
        auto_selected_prev = auto_selected;
        List<PathPlannerPath> auto_paths = PathPlannerAuto.getPathGroupFromAutoFile(auto_selected);
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
        {
          starting_pose = auto_paths.get(0).flipPath().getStartingHolonomicPose().get();
        }
        else
        {
          starting_pose = auto_paths.get(0).getStartingHolonomicPose().get();
        }
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : auto_paths) 
        {
          if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) path = path.flipPath();
          poses.addAll(path.getAllPathPoints().stream().map(
            point -> new Pose2d(point.position.getX(), 
            point.position.getY(), 
            new Rotation2d()))
            .collect(Collectors.toList()));
        }

        m_robotContainer.drivetrain.odometryField.getObject("starting").setPose(starting_pose);
        m_robotContainer.drivetrain.odometryField.getObject("trajectoryAuto").setPoses(poses);
      }
      } catch (Exception e) {
        e.printStackTrace();
    }

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
        m_robotContainer.teleopInit = false;

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    SmartDashboard.putBoolean("hub active", true);
    m_robotContainer.teleopInit = false;
  }
  Alert test = new Alert("Test!", AlertType.kError);
  @Override
  public void teleopInit() {
    test.set(true);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit = true;
    m_robotContainer.drivetrain.odometryField.getObject("starting").setPose(new Pose2d(-100,-100,Rotation2d.kZero));
    m_robotContainer.drivetrain.odometryField.getObject("trajectoryAuto").setPose(new Pose2d(-100,-100,Rotation2d.kZero));
  }

  double matchTime;
  String gameData;
  boolean redFirstShift;
  boolean hubActive;
  boolean WeFirstShift;
  double remainingShiftTime;
  Optional<Alliance> alliance;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
        m_robotContainer.teleopInit = false;

    alliance = DriverStation.getAlliance();
    matchTime = DriverStation.getMatchTime();
    gameData = DriverStation.getGameSpecificMessage();
    
    if(gameData.isEmpty() || alliance.isEmpty())
    {
      hubActive = true;
    }
    else
    {
      if(gameData.charAt(0) == 'R')
      {
        redFirstShift = false;
      } 
      else 
      {
        redFirstShift = true;
      }

      if((alliance.get() == Alliance.Red && redFirstShift) || (alliance.get() == Alliance.Blue && !redFirstShift))
      {
        WeFirstShift = true;
      }
      else
      {
        WeFirstShift = false;
      }


      if (matchTime > 130) {
        // Transition shift, hub is active.
        remainingShiftTime = matchTime - 130;
        hubActive = true;
      } 
      else if (matchTime > 105) 
      {
        // Shift 1
        remainingShiftTime = matchTime - 105;
        hubActive = WeFirstShift;
      } 
      else if (matchTime > 80) 
      {
        // Shift 2
        remainingShiftTime = matchTime - 80;
        hubActive = !WeFirstShift;
      } 
      else if (matchTime > 55) 
      {
        // Shift 3
        remainingShiftTime = matchTime - 55;
        hubActive = WeFirstShift;
      } 
      else if (matchTime > 30) 
      {
        // Shift 4
        remainingShiftTime = matchTime - 30;
        hubActive = !WeFirstShift;
      } 
      else 
      {
        // End game, hub always active.
        remainingShiftTime = matchTime;
        hubActive = true;
      }
    }
    SmartDashboard.putNumber("shift timer", remainingShiftTime);
    SmartDashboard.putBoolean("hub active", hubActive);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
