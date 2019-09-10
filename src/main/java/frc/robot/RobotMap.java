/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // CAN Addresses - Talon
  public static int starboardAddress = 2; // Last Configured: Never
  public static int portAddress = 1; // Last Configured: Never

  // Driver Stick Information
  public static Joystick driverStick; // Can list more than one joystick here
  public static int driverStickPort = 0; // The driver station item slot this controler resides in
  public static int driverThroAxis = 1; // The axis for thro
  public static int driverYawAxis = 2; // The axis for yaw

  // Drive motor Controlers
  public static TalonSRX starboardMotor;
  public static TalonSRX portMotor;
  public static TalonSRX strafeMotor;

  // Other
  public static int invertSteering = -1;  // Invert steering
  public static int invertStrafe = 1; // Invert strafe
}