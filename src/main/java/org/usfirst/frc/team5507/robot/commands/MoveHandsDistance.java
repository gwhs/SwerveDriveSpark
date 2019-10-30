/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import com.revrobotics.ControlType;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class MoveHandsDistance extends Command {
  private double pos;
  public MoveHandsDistance(double pos) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    this.pos = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_climber.NEncoder3.setPosition(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_climber.moveHandsPosition(pos);
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
