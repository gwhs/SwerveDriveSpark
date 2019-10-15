/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team5507.robot.Robot;

import com.kauailabs.navx.frc.AHRS;

public class FrontArmDown extends Command {
  public FrontArmDown() {
    requires(Robot.m_climber);
  };
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   /*  if (mNavX.getAngle() < 30) //angle is not accurate, just a placeholder
    {
      Robot.m_climber.moveArm1(1); //speed not accurate, just a placeholder too ;D
    }
    else
    {
      Robot.m_climber.stopArm1();
    } */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
