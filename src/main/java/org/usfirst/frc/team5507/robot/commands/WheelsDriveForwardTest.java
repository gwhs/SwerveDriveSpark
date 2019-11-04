/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class WheelsDriveForwardTest extends Command {
  private Timer time = new Timer();
  private static boolean isEnded;
  private double driveTime = 1;
  private double armSpeed = .7;
  private double d1;
  private double startAngle;
  private double speed;
  public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
  public static final double TOTAL_SENSOR_POS = 1024;
  public static final double DISTANCE = WHEEL_CIRCUMFERENCE / TOTAL_SENSOR_POS;
 
  
  public WheelsDriveForwardTest(double d, double speed) {
      isEnded = false;
      requires(Robot.swerveDriveSubsystem);
      d1 = DISTANCE * d * 12;
      this.speed = speed;
  }

  public void driveForward() 
  {
    if(time.get() < driveTime)
    {
      Robot.m_climber.moveHand1(armSpeed);
    }
    Robot.m_climber.stop();
    System.out.println("Field orientation is " + (Robot.swerveDriveSubsystem.isFieldOriented() ? "true" : "false"));
    Robot.swerveDriveSubsystem.driveForwardDistance(d1, startAngle, 0.5);
    isEnded = true;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.reset();
    time.start();
    startAngle = Robot.swerveDriveSubsystem.getNavX().getYaw();
    Robot.swerveDriveSubsystem.resetAllEncoders();
    Robot.swerveDriveSubsystem.setFieldOriented(false);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveForward();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isEnded;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    isEnded = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
