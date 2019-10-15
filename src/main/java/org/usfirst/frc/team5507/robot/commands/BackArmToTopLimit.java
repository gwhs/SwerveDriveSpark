/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class BackArmToTopLimit extends Command {
  private static boolean isEnded;

  public BackArmToTopLimit()
  {
    isEnded = false;
  }
  
  public void backArmToTopLimit()
  {
    if (Robot.m_climber.getReverseLimit() == true)  //Top Limit
    {
      isEnded = true;
    }
    else
    {
      Robot.m_climber.moveArm2(-.1);
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isEnded = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    backArmToTopLimit();
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
