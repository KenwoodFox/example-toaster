/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ros_auto;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Commands
  Command autonomous;

  // ROS Variables
  NetworkTableEntry robotX; // Current x, y and heading of the robot
  NetworkTableEntry robotY;
  NetworkTableEntry robotHeading;

  NetworkTableEntry coprocessorPort; // For tank drive
  NetworkTableEntry coprocessorStarboard;
  NetworkTableEntry rosTime; // Is ros time (slow estimate)

  public static Drivetrain drivetrain = new Drivetrain();
  public static OI robotOI;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Define Commands
    robotOI = new OI();
    autonomous = new ros_auto();
    
    // Define Joysticks
    RobotMap.driverStick = new Joystick(RobotMap.driverStickPort);  // Define driver stick

    // Define motors
    RobotMap.starboardMotor = new TalonSRX(RobotMap.starboardAddress); // Define starboard motor
    RobotMap.portMotor = new TalonSRX(RobotMap.portAddress); // Define port motor

    // Setup Network Tables
    NetworkTableInstance inst = NetworkTableInstance.getDefault(); // Get the default instance of network tables on the rio
    NetworkTable rostable = inst.getTable("ros"); // Get the table ros

    // Define ROS vars
    robotX = rostable.getEntry("robotX");
    robotY = rostable.getEntry("robotY");
    robotHeading = rostable.getEntry("robotHeading");
    coprocessorPort = rostable.getEntry("coprocessorPort");
    coprocessorStarboard = rostable.getEntry("coprocessorStarboard");
    rosTime = rostable.getEntry("rosTime");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double portEncoder = RobotMap.portMotor.getSelectedSensorPosition(); // Get the encoder value of the port encoder
    double startboardEncoder = RobotMap.starboardMotor.getSelectedSensorPosition(); // Ge the encoder value of the starboard encoder
    SmartDashboard.putNumber("Port", portEncoder); // Publish the values to smartdashboard
    SmartDashboard.putNumber("Starboard", startboardEncoder);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function called on autonomous start.
   */
  @Override
  public void autonomousInit() {
    autonomous.start(); // Start the autonomous (runs command ros_auto())
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run(); // Actually not sure what this line does
  }

  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    Drivetrain.flyByWireA(RobotMap.starboardMotor, RobotMap.portMotor, RobotMap.driverStick, Constants.turnDampener, Constants.throDampener); // Using the driver control stick, fly by wire
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
