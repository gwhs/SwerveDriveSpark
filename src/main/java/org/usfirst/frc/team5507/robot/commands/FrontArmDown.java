/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team5507.robot.Robot;

import com.kauailabs.navx.frc.AHRS;

public class FrontArmDown extends Command {
  private boolean isEnded;
  private Timer time = new Timer();

  public FrontArmDown() {
    requires(Robot.m_climber);
    isEnded = false;
  };
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.reset();
    time.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   // if (time.get() < 3) //angle is not accurate, just a placeholder
   if (Robot.swerveDriveSubsystem.mNavX.getRoll() < 6) 
   {
      Robot.m_climber.moveArm1(-1); //speed not accurate, just a placeholder too ;D
    }
    else
    {
      Robot.m_climber.stopArm1();
      isEnded = true;
    }
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
//comment
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
