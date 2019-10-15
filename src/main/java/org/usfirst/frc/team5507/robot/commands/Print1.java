/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.Timer;

public class Print1 extends Command {
  private Timer clock = new Timer();
  private boolean end;
  public Print1() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    end = false; 
  }

  // Called just before this Command runs the first time
  @Override //print "hello"
  protected void initialize() {
    System.out.println("Hello");
    clock.start();
    end = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override //print time for 3 seconds using TIMER class
  protected void execute() {
    if(clock.get() > 1)
    {
     System.out.println(clock.get());
    }
    if(clock.get() > 3)
    {
      end = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return end;
  }

  // Called once after isFinished returns true
  @Override //print "world"
  protected void end() {
    System.out.println("World");
    clock.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
