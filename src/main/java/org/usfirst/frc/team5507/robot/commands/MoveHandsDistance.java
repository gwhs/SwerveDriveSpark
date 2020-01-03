/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class MoveHandsDistance extends Command {
  private double pos;
  private double curPos;
  public MoveHandsDistance(double pos) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    this.pos = pos + Robot.m_climber.getHandEncoder().getPosition();
    curPos = Robot.m_climber.getHandEncoder().getPosition();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_climber.moveHandsPosition(pos);
    curPos = Robot.m_climber.getHandEncoder().getPosition();
    System.out.println((pos -2) < curPos);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (pos -2) < curPos; //Expected - Range is less than Currrent Position
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climber.stopHand();
    System.out.println("End of moveHandsDistance");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
