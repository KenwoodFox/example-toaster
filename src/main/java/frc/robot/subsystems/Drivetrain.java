/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * This subsystem interfaces to the drivetrain
 */
public class Drivetrain extends Subsystem {
  /**
   * flyByWireA uses no PID control and is fully manual
   * For manual driver stick imput only
   */
  public static void flyByWireA(TalonSRX starboard, TalonSRX port, Joystick DriverJoystick, double turnDampener, double throDampener, boolean verbose)
  {
    double thro = throDampener * DriverJoystick.getRawAxis(RobotMap.driverThroAxis); // Populate thro throAxis
    double yaw = turnDampener * DriverJoystick.getRawAxis(RobotMap.driverYawAxis); // Populate with yawAxis

    starboard.set(ControlMode.PercentOutput, (-1 * thro) - (yaw * RobotMap.invertSteering));  // From the inverse of thro, subtract yaw
    port.set(ControlMode.PercentOutput, thro - (yaw * RobotMap.invertSteering));  // subtract yaw from thro

    if(verbose == true){
      SmartDashboard.putNumber("Thro", thro);
      SmartDashboard.putNumber("Yaw", yaw);
    }
  } 
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
