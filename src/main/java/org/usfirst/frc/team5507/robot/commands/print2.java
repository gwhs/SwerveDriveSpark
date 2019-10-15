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

public class print2 extends Command {
  private Timer clock2 = new Timer(); 
  private boolean end2;
  public print2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    end2 = false;
  }

  // Called just before this Command runs the first time
  @Override //print "goodbye"
  protected void initialize() {
    System.out.println("goodbye");
    end2 = false;
    clock2.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override //print nothin end this after 3 secodns using TIMER class
  protected void execute() {
    if(clock2.get() > 3)
    {
      end2 = true;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return end2;
  }

  // Called once after isFinished returns true
  @Override // print "world"
  protected void end() {
    System.out.println("world");
    clock2.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
