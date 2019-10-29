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

public class FrontArmMatch extends Command {
  private boolean isEnded;
  private double angle;
  private Timer test = new Timer();
  private final double SPEED_CONSTANT = 0.1;
  private final double ROLL_LIMIT = 1.5;
    
  public FrontArmMatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    isEnded = false;
    angle = (double)Robot.swerveDriveSubsystem.mNavX.getRoll();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    test.reset();
    test.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    angle = (double)Robot.swerveDriveSubsystem.mNavX.getRoll();
    System.out.println("Angle: " + angle);
    double frontArmSpeed;
    double handSpeed;
    if(Robot.swerveDriveSubsystem.mNavX.getRoll() > ROLL_LIMIT)
    {
      frontArmSpeed = .1;
      handSpeed = 0;
    }
    else
    {
      frontArmSpeed = SPEED_CONSTANT*Robot.swerveDriveSubsystem.mNavX.getRoll();
      handSpeed = .15;
    }
    System.out.println("Front arm Speed: " + frontArmSpeed);
    Robot.m_climber.moveArm1(frontArmSpeed);
    Robot.m_climber.moveHand1(handSpeed);
    isEnded = Robot.m_climber.getReverseLimit();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isEnded;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climber.stop();
    isEnded = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
