// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
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
  private PathPlannerPath currentPath;
  List<PathPlannerPath> pathList;
  private List<Waypoint> waypointList;
  private List<Pose2d> poseList;
  //private List<PathPlannerPath> pathList;


  public Robot() {
    PathPlannerTrajectory p_trajectory;
    Pose2d currentPose = new Pose2d();
    try {
       currentPath = PathPlannerPath.fromPathFile("top-zero_Bottom");
        } catch (IOException e) {
          System.out.println("IO exception Blue45_i :");e.printStackTrace();
          } catch (ParseException e) {
          System.out.println("ParseException Blue45_i :");e.printStackTrace();
        }  catch (FileVersionException e) {
          System.out.println("FileVersionException Blue45_i :");e.printStackTrace();
        }
    // try {
    //   pathList= PathPlannerAuto.getPathGroupFromAutoFile("auto1");
    //     } catch (IOException e) {
    //       System.out.println("IOException :");e.printStackTrace();
    //       } catch (ParseException e) {
    //       System.out.println("ParseException  :");e.printStackTrace();
    //     }

        
        // points are not of value 
        //  SmartDashboard.putString("getAllPathPoints", currentPath.getAllPathPoints().toString());
        SmartDashboard.putString("getPathPoses",currentPath.getPathPoses().toString());
        SmartDashboard.putString("getStartingHolonomicPose",currentPath.getStartingHolonomicPose().toString());
        
    // System.out.println("currentPath getAllPathPoints() " + currentPath.getAllPathPoints().toString());
    // System.out.println("currentPath getAllPathPoints() " + currentPath.getAllPathPoints().toString());
    
    System.out.println("currentPath getPathPoses() " + currentPath.getPathPoses().toString() );
    System.out.println("***********");
    System.out.println("           ");
   
    for (int j = 0; j < currentPath.getWaypoints().size(); j++) {
     System.out.println("getWayPoints() :" + currentPath.getWaypoints().get(j).toString() );
    }

    System.out.println("***********");
    System.out.println("currentPath getStartingHolonomicPose() " + currentPath.getStartingHolonomicPose().toString() );
    
    SmartDashboard.putData("Field", m_field);
    //m_field.setRobotPose(currentPose);
    m_field.getObject("tarject").setPoses(currentPath.getPathPoses());
    // m_field.setRobotPose(currentPath.getPathPoses().get(0));
    // m_field.setRobotPose(currentPath.getPathPoses().get(1));
    // m_field.setRobotPose(currentPath.getPathPoses().get(2));
    // m_field.setRobotPose(currentPath.getPathPoses().get(3));

    // for (int i = 0; i < currentPath.getPathPoses().size(); i++) {
    //   String str= "Position-" + i;
    //   m_field.getObject(str).setPose(currentPose);
    //   //setPose(m_field.setRobotPose(currentPath.getPathPoses().get(i)) );
    // }


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
