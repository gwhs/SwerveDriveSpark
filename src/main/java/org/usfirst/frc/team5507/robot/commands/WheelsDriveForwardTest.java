/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import com.revrobotics.ControlType;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class WheelsDriveForwardTest extends Command {
  private double driveTime = 1;
  private double d1;
  private double startAngle;
  public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
  public static final double TOTAL_SENSOR_POS = 64;
  public static final double DISTANCE = WHEEL_CIRCUMFERENCE / TOTAL_SENSOR_POS;
  private double initAngle0;
  private double initAngle1;
  private double initAngle2;
  private double initAngle3;
  private double curPos;
  private double pos1;
  private double pos2;
  private double pos3;
  private double pos4;
  private double angle;
  private double targetPos;

  public WheelsDriveForwardTest(double d, double targetAngle) { 
      requires(Robot.swerveDriveSubsystem);
       // inches to ticks
       d1 = d;
       targetPos = (d1* 8.5714)/(4*Math.PI);
      System.out.println("Starting Drive Forward");
    angle = targetAngle;
    
      // initAngle0 = Robot.swerveDriveSubsystem.m1.getTargetAngle();
      // initAngle1 = Robot.swerveDriveSubsystem.m2.getTargetAngle();
      // initAngle2 = Robot.swerveDriveSubsystem.m3.getTargetAngle();
      // initAngle3 = Robot.swerveDriveSubsystem.m4.getTargetAngle();
  }

  public void driveForward() 
  {
    //System.out.println("Field orientation is " + (Robot.swerveDriveSubsystem.isFieldOriented() ? "true" : "false"));
    // Robot.swerveDriveSubsystem.driveForwardDistance(d1, startAngle);
    //isEnded = false;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    Robot.swerveDriveSubsystem.m1.getPIDController().setReference(pos1 + d1, ControlType.kPosition);
    Robot.swerveDriveSubsystem.m2.getPIDController().setReference(pos2 + d1, ControlType.kPosition);
    Robot.swerveDriveSubsystem.m3.getPIDController().setReference(pos3 + d1, ControlType.kPosition);
    Robot.swerveDriveSubsystem.m4.getPIDController().setReference(pos4 + d1, ControlType.kPosition);
    System.out.println("Driving Wheels Forward");
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    pos1 = Robot.swerveDriveSubsystem.m1.getDriveMotor().getEncoder().getPosition();
    curPos = Robot.swerveDriveSubsystem.m1.getDriveMotor().getEncoder().getPosition();
    pos2 = Robot.swerveDriveSubsystem.m2.getDriveMotor().getEncoder().getPosition();
    pos3 = Robot.swerveDriveSubsystem.m3.getDriveMotor().getEncoder().getPosition();
    pos4 = Robot.swerveDriveSubsystem.m4.getDriveMotor().getEncoder().getPosition();
    startAngle = Robot.swerveDriveSubsystem.getNavX().getYaw();
    Robot.swerveDriveSubsystem.resetAllEncoders();
    //Robot.swerveDriveSubsystem.setFieldOriented(false);
    Robot.swerveDriveSubsystem.driveForwardDistance(d1,angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    curPos = Robot.swerveDriveSubsystem.m1.getDriveMotor().getEncoder().getPosition();
    boolean ya = (Math.abs(pos1) + targetPos - 3) < Math.abs(curPos);
    System.out.println( "Target Pos: " + (Math.abs(pos1) + targetPos - 3) +  "Actual Pos: " + Math.abs(curPos) + " Drive Ended: " + ya);
    // Robot.swerveDriveSubsystem.m1.setTargetAngle(startAngle);
    // Robot.swerveDriveSubsystem.m2.setTargetAngle(startAngle);
    // Robot.swerveDriveSubsystem.m3.setTargetAngle(startAngle);
    // Robot.swerveDriveSubsystem.m4.setTargetAngle(startAngle);
    //driveForward();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(pos1) + targetPos - 3) < Math.abs(curPos);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Ending Drive Forward");
    Robot.swerveDriveSubsystem.setFieldOriented(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
