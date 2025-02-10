// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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
  private static ElevatorManualMove elevatorManual = new ElevatorManualMove(elevator);
  
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

    try {
      pathList= PathPlannerAuto.getPathGroupFromAutoFile("auto1");
        } catch (IOException e) {
          System.out.println("IOException :");e.printStackTrace();
          } catch (ParseException e) {
          System.out.println("ParseException  :");e.printStackTrace();
        }

    //NamedCommands.registerCommand("ElevatorUp",ElevatorUp );

    flipCurrentPath = currentPath.flipPath();
    //System.out.println("currentPath getPathPoses() " + currentPath.flipPath() );
    
    // System.out.println("currentPath getIdealTrajectory(config) " + currentPath.getIdealTrajectory(config).toString() );
    try {
      pointList= currentPath.getAllPathPoints();
      } catch (Exception e) { 
        System.out.println("error" + e); 
      }

      start = currentPath.getWaypoints().get(0).anchor().div(1);
      end = currentPath.getWaypoints().get(1).anchor().div(1);

      startRotation = currentPath.getIdealStartingState().rotation();
      endRotation = currentPath.getGoalEndState().rotation();

        // System.out.println("getWayPoints(0) :" + currentPath.getWaypoints().get(0).anchor().div(1) );
        // System.out.println("getWayPoints(1) :" + currentPath.getWaypoints().get(1).anchor().div(1) );
    
      SmartDashboard.putData("Field", m_field);
      //publish paths to Field2d 
        //  m_field.getObject("primary").setPoses(currentPath.getPathPoses());
        m_field.getObject("flipped").setPoses(flipCurrentPath.getPathPoses()); 
        //  m_field.getObject("mirrored").setPoses(flipCurrentPath.mirrorPath().getPathPoses()); 
    
        poseList = currentPath.getPathPoses();
  
      // remove the LAST and FIRST entree without modifying original pointList
          trimList.addAll(pointList);
          trimList.remove(pointList.size()-1);    // LAST removed
          trimList.remove(0 );              // FIRST removed

          // ignore first point and last point (size - 1 )
          //  for (int i=1; i < trimList.size() - 1 ; i++ ) {  // was skipping FIRST and LAST
          for (int i =0; i < trimList.size() ; i++ ) {         // process all List elements
              System.out.println("pose" + i + " :" + pointList.get(i).position.toString() );
              midWaypoints.add(pointList.get(i).position );
          }

        startPose = new Pose2d( start , startRotation);
        endPose = new Pose2d(end , endRotation);

        m_field.getObject("startPose").setPose(startPose);
        m_field.getObject("endPose").setPose(endPose);

        // alternate start pose2d creation
            // startPose = new Pose2d( 
            //   currentPath.getPathPoses().get(0).getX(),
            //   currentPath.getPathPoses().get(0).getY(),
            //   startRotation);

        // alternate end pose2d creation
            // endPose = new Pose2d( 
            //    currentPath.getPathPoses().get(0).getX(),
            //    currentPath.getPathPoses().get(0).getY(),
            //    startRotation);

            for (int index = 0; index < midWaypoints.size(); index++) 
            {
              m_field.getObject("points_"+ index).setPose(midWaypoints.get(index).getX(), 
                midWaypoints.get(index).getX(), 
                new Rotation2d() );
            }

     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        startPose, 
        midWaypoints, 
        endPose, 
        new TrajectoryConfig(3, 2.9) );

      // System.out.println( "trajectory: " + trajectory.toString() );
        }
      

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
}
