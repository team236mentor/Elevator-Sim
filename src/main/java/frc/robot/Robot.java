// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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

  private List<Translation2d> interiorWaypoints;
  private List<Waypoint> waypointList;
  private List<Pose2d> poseList;
  private PathPlannerTrajectory p_trajectory;
  private Pose2d currentPose = new Pose2d();
  private Pose2d startPose, endPose;

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

        
    // don't getAllPathPoints: is not of any value      currentPath.getAllPathPoints() 
    //
    System.out.println("currentPath getPathPoses() " + currentPath.getPathPoses().toString());
    System.out.println("currentPath getStartingHolonomicPose() " + currentPath.getStartingHolonomicPose().toString());
    
    flipCurrentPath = currentPath.flipPath();
    //System.out.println("currentPath getPathPoses() " + currentPath.flipPath() );
    
    System.out.println("currentPath getPathPoses() " + currentPath.getPathPoses().toString() );
    System.out.println("*** poses *****");
    // System.out.println("currentPath getIdealTrajectory(config) " + currentPath.getIdealTrajectory(config).toString() );
   
    // for (int j = 0; j < currentPath.getWaypoints().size()-1; j++) {
    //  System.out.println("getWayPoints() :" + currentPath.getWaypoints().get(j).toString() );
    // }

   // System.out.println("***********");
   // System.out.println("currentPath getStartingHolonomicPose() " + currentPath.getStartingHolonomicPose().toString() );
    
    SmartDashboard.putData("Field", m_field);
      
    //publish paths to Field2d 
      m_field.getObject("primary").setPoses(currentPath.getPathPoses());
    //  m_field.getObject("flipped").setPoses(flipCurrentPath.getPathPoses()); 
      m_field.getObject("mirrored").setPoses(flipCurrentPath.mirrorPath().getPathPoses()); 
    
    // not clear how to get get just X and Y waypoint values for WPILIB math tarjectory
      // List<getWayPoint> ; 
      // List<Waypoint> waypointList = currentPath.getPathPoses();
  
      // System.out.println("waypointSize: " + waypointList);

        //for (int index = 0; index < waypointList.size()-1; index++) {
        //do we need to skip start and end points?
        int size = currentPath.getPathPoses().size();
        for (int i = 0; i < size; i++) {
          // currentPath.getPathPoses().remove(1);
          // Translation2d way = new Translation2d(
            System.out.println("currentPath pathPoint-" + i + currentPath.get(i).getPoint());
          // currentPath.get(i).getPoint(), 6.03); 
          // interiorWaypoints.add(way);
        }
        Translation2d way = new Translation2d(5.39, 6.03); 
        interiorWaypoints.add(way);
        //}

        startRotation = currentPath.getIdealStartingState().rotation();
        endRotation = currentPath.getGoalEndState().rotation();
        startPose = new Pose2d( 
          currentPath.getPathPoses().get(0).getX(),
          currentPath.getPathPoses().get(0).getY(),
          startRotation);
        endPose = new Pose2d( 
           currentPath.getPathPoses().get(0).getX(),
           currentPath.getPathPoses().get(0).getY(),
           startRotation);

     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        startPose, 
        interiorWaypoints, 
        endPose, 
        new TrajectoryConfig(3, 2.9));

      System.out.println("**** trajectory *****");
      System.out.println("trajectory: " + trajectory.toString() );
      System.out.println("**** trajectory *****");
      // m_field.setRobotPose(currentPose);
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
