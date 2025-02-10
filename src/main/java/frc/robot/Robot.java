// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  private final Joystick m_joystick = new Joystick(Constants.kJoystickPort);
  private final Elevator m_elevator = new Elevator();
  

  private static Field2d m_field = new Field2d();
  private PathPlannerPath currentPath,flipCurrentPath;
  private Rotation2d startRotation, endRotation;

  private List<PathPlannerPath> pathList;
  private RobotConfig config;

  private List<Translation2d> interiorWaypoints = null;
  private List<Waypoint> waypointList = null;
  private List<Pose2d> poseList = null;

  private List<PathPoint> pointList= null;
  private List<PathPoint> trimList= null;

  private PathPlannerTrajectory p_trajectory;
  private Pose2d currentPose = new Pose2d();
  private Pose2d startPose, endPose;
  private Translation2d start, end;
  private List<Translation2d> midWaypoints = null;


  public Robot() {

    try {
       currentPath = PathPlannerPath.fromPathFile("2Left45_Reef-K");
        } catch (IOException e) {
          System.out.println("IO exception 2Left45_Reef-K :");e.printStackTrace();
        } catch (ParseException e) {
          System.out.println("ParseException 2Left45_Reef-K :");e.printStackTrace();
        }  catch (FileVersionException e) {
          System.out.println("FileVersionException 2Left45_Reef-K :");e.printStackTrace();
        }        
 
    try {
      config = RobotConfig.fromGUISettings();
        } catch (IOException e) {
          System.out.println("IO exception fromGUISettings :");e.printStackTrace();
        } catch (ParseException e) {
          System.out.println("ParseException fromGUISettings :");e.printStackTrace();
        }

    // try {
    //   pathList= PathPlannerAuto.getPathGroupFromAutoFile("auto1");
    //     } catch (IOException e) {
    //       System.out.println("IOException :");e.printStackTrace();
    //       } catch (ParseException e) {
    //       System.out.println("ParseException  :");e.printStackTrace();
    //     }

        
    // don't getAllPathPoints: is not of any value  ??     
    
    System.out.println("currentPath getPathPoses() " + currentPath.getPathPoses().toString());
    //System.out.println("currentPath getStartingHolonomicPose() " + currentPath.getStartingHolonomicPose().toString());
    
    flipCurrentPath = currentPath.flipPath();
    //System.out.println("currentPath getPathPoses() " + currentPath.flipPath() );
    
    // System.out.println("currentPath getIdealTrajectory(config) " + currentPath.getIdealTrajectory(config).toString() );
    try {
      pointList= currentPath.getAllPathPoints();
      } catch (Exception e) { 
        System.out.println("error" + e); 
      }

      
    System.out.println("getWayPoints(0) :" + currentPath.getWaypoints().get(0).anchor().div(1) );
    System.out.println("getWayPoints(1) :" + currentPath.getWaypoints().get(1).anchor().div(1) );

   // System.out.println("***********");
   // System.out.println("currentPath getStartingHolonomicPose() " + currentPath.getStartingHolonomicPose().toString() );
    
    SmartDashboard.putData("Field", m_field);
      //publish paths to Field2d 
      m_field.getObject("primary").setPoses(currentPath.getPathPoses());
      m_field.getObject("flipped").setPoses(flipCurrentPath.getPathPoses()); 
      // m_field.getObject("mirrored").setPoses(flipCurrentPath.mirrorPath().getPathPoses()); 
    
    // not clear how to get get just X and Y waypoint values for WPILIB math tarjectory
      // List<getWayPoint> ; 
      poseList = currentPath.getPathPoses();
  
      // System.out.println("waypointSize: " + waypointList);

        //for (int index = 0; index < waypointList.size()-1; index++) {
        //do we need to skip start and end points?

        trimList = pointList;
        // remove the LAST and FIRST entree
          // trimList.remove(pointList.size()-1);
          // trimList.remove(0 );

        int size = trimList.size();
            for (int i =1; i < size - 1 ; i++ ) {
              System.out.println("pose(1) :" + trimList.get(i).toString() );
              midWaypoints.add(trimList.get(i).position );
              
          }

        // TODO Translation2d way = start; 
        // interiorWaypoints =  new ArrayList<Translation2d>();
        // interiorWaypoints.add(way);

        startRotation = currentPath.getIdealStartingState().rotation();
        endRotation = currentPath.getGoalEndState().rotation();

        start = currentPath.getWaypoints().get(0).anchor().div(1);
        end = currentPath.getWaypoints().get(1).anchor().div(1);
        
        //TODO 
        // startPose = new Pose2d( start , startRotation);
        
            // startPose = new Pose2d( 
            //   currentPath.getPathPoses().get(0).getX(),
            //   currentPath.getPathPoses().get(0).getY(),
            //   startRotation);
        
        //TODO 
        //endPose = new Pose2d(end , endRotation);
        
            // endPose = new Pose2d( 
            //    currentPath.getPathPoses().get(0).getX(),
            //    currentPath.getPathPoses().get(0).getY(),
            //    startRotation);

    //  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //     startPose, 
    //     interiorWaypoints, 
    //     endPose, 
    //     new TrajectoryConfig(3, 2.9));

    //   System.out.println("**** trajectory *****");
    //   System.out.println("trajectory: " + trajectory.toString() );
    //   System.out.println("**** trajectory *****");

    //  m_field.setRobotPose(end);
  }

  @Override
  public void robotPeriodic() {
    // Update the telemetry, including mechanism visualization, regardless of mode.
    m_elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    m_elevator.simulationPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    if (m_joystick.getTrigger()) {
      // Here, we set the constant setpoint of 0.75 meters.
      m_elevator.reachGoal(Constants.kSetpointMeters);
    } else {
      // Otherwise, we update the setpoint to 0.
      m_elevator.reachGoal(0.0);
    }
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_elevator.stop();
  }

  @Override
  public void close() {
    m_elevator.close();
    super.close();
  }
}
