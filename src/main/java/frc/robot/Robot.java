// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command.ElevatorManualMove;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  
  // create subsystems
  private final Joystick m_joystick = new Joystick(Constants.kJoystickPort);
  private final static Elevator elevator = new Elevator();

  // create Commands
  // private static ElevatorManualMove elevatorManual = new ElevatorManualMove(elevator);
  
  private static Field2d m_field = new Field2d();

  private PathPlannerPath currentPath,flipCurrentPath;

  private List<PathPlannerPath> pathList;
  private Rotation2d startRotation, endRotation;
  private List<PathPoint> pointList = new ArrayList<>();
  private List<PathPoint> trimList = new ArrayList<>();
  private Pose2d startPose, endPose;
  private List<Pose2d> poseList = new ArrayList<>();
  private Translation2d start, end;
  public List<Translation2d> midWaypoints = new ArrayList<>();

  private RobotConfig config;
    
// private List<Translation2d> interiorWaypoints = null;
// private List<Waypoint> waypointList = null;

  public Robot() {
    
    // PATHPLANNER - read specific path into currentpath from pathplanner file 
    // keep this and other reads early in Robot 
    try {
       currentPath = PathPlannerPath.fromPathFile("2Left45_Reef-K");
        } catch (IOException e) {     //  could convert to single Exception catch
          System.out.println("IO exception 2Left45_Reef-K :");e.printStackTrace();
        } catch (ParseException e) {
          System.out.println("ParseException 2Left45_Reef-K :");e.printStackTrace();
        }  catch (FileVersionException e) {
          System.out.println("FileVersionException 2Left45_Reef-K :");e.printStackTrace();
        }        
 
    // PATHPLANNER - required to run pathplanner Auto methodologies not required if running wpilib trajectories
    // can keep in RobotsConfig or Robots for this simple command class layout
    try {
      config = RobotConfig.fromGUISettings();
        } catch (IOException e) {
          System.out.println("IO exception fromGUISettings :");e.printStackTrace();
        } catch (ParseException e) {
          System.out.println("ParseException fromGUISettings :");e.printStackTrace();
        }

    // PATHPLANNER - reading a multipath pathplanner auto as pathList 
    // TODO Evaluate how this is implements parrallel series commands with NamedCommand
    //    NamedCommands.registerCommand("ElevatorUp",ElevatorUp );
          try {
            pathList= PathPlannerAuto.getPathGroupFromAutoFile("auto1");
              } catch (IOException e) {
                System.out.println("IOException :");e.printStackTrace();
                } catch (ParseException e) {
                System.out.println("ParseException  :");e.printStackTrace();
              }

      SmartDashboard.putData("Field", m_field);
      //publish paths to Field2d 
        
        m_field.getObject("primary").setPoses(currentPath.getPathPoses());
        // m_field.getObject("mirror_flip").setPoses(currentPath.flipPath().mirrorPath().getPathPoses()); 
        
        m_field.getObject("trajectory").setTrajectory(ChangePathPlannerPathtoTrajectory(currentPath,true ) );
        
        // overlay starting ending pose with correct angle
        m_field.getObject("start").setPose( new Pose2d(currentPath.getWaypoints().get(0).anchor(), currentPath.getIdealStartingState().rotation() ) ); 
        m_field.getObject("end").setPose( new Pose2d(currentPath.getWaypoints().get(1).anchor(), currentPath.getGoalEndState().rotation() ) ); 
        
        // overlay traget mirror path , with starting ending pose with correct angle
        // m_field.getObject("trajectory").setTrajectory(ChangePathPlannerPathtoTrajectory(currentPath.mirrorPath(),true ) );
        // m_field.getObject("start").setPose( new Pose2d(currentPath.mirrorPath().getWaypoints().get(0).anchor(), currentPath.mirrorPath().getIdealStartingState().rotation() ) ); 
        // m_field.getObject("end").setPose( new Pose2d(currentPath.mirrorPath().getWaypoints().get(1).anchor(), currentPath.mirrorPath().getGoalEndState().rotation() ) ); 
    }     // end of constructor

  @Override
  public void robotPeriodic() {
    // Update the telemetry, including mechanism visualization, regardless of mode.
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    elevator.simulationPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    if (m_joystick.getTrigger()) {
      // Here, we set the constant setpoint of 0.75 meters.
      elevator.reachGoal(Constants.kSetpointMeters);
    } else {
      // Otherwise, we update the setpoint to 0.
      elevator.reachGoal(0.0);
    }
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    elevator.stop();
  }

  @Override
  public void close() {
    elevator.close();
    super.close();
  }

  // method to convert pathplannerpath to tarjectory 
  public Trajectory ChangePathPlannerPathtoTrajectory(PathPlannerPath path,boolean reduce) {
    /* 
     * starting to convert a specific pathPlannerPath to wpilib trajectory 
     * this should be method or own utility class for conversion    */
    try {
      pointList= path.getAllPathPoints();
      } catch (Exception e) { 
        System.out.println("error" + e); 
      }

      start = path.getWaypoints().get(0).anchor().div(1);
      end = path.getWaypoints().get(1).anchor().div(1);

      startRotation = path.getIdealStartingState().rotation();
      endRotation = path.getGoalEndState().rotation();
      // trouble shooting 
        // System.out.println("getWayPoints(0) :" + path.getWaypoints().get(0).anchor().div(1) );
        // System.out.println("getWayPoints(1) :" + path.getWaypoints().get(1).anchor().div(1) );
    
        poseList = path.getPathPoses();
  
      // remove the LAST and FIRST entree without modifying original pointList
          trimList.addAll(pointList);
          trimList.remove(pointList.size()-1);    // LAST removed
          trimList.remove(0 );              // FIRST removed

      // ignore first point and last point (size - 1 )
          for (int i =0; i < trimList.size(); i++ ) {         // process all List elements
            //  System.out.println("pose" + i + " :" + trimList.get(i).position.toString() );
             
            //  remove the odd entrees 
              if (reduce && i % 10 != 0) { 
                System.out.println("remove" + i + " :" + trimList.get(i).position.toString() );
                trimList.remove(i); 
              } else {
                midWaypoints.add(trimList.get(i).position );
                System.out.println("traj" + i + " :" + trimList.get(i).position.toString() );
              } 
          }

        startPose = new Pose2d( start , startRotation);
        endPose = new Pose2d(end , endRotation);

    //  Creates the "wpilib trajectory" could be in a function to return, or its own utility
     Trajectory traj = TrajectoryGenerator.generateTrajectory(
        startPose, 
        midWaypoints, 
        endPose, 
        new TrajectoryConfig(3, 2.9) );
            return traj;
        }

}
