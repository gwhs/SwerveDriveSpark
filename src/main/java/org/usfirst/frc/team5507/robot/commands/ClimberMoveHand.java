/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberMoveHand extends Command {
private double speed;

  public ClimberMoveHand(double speed) {
   requires(Robot.m_climber);
    this.speed = speed;
  }

  @Override
  protected void initialize() {
    //Robot.m_climber.NEncoder3.setPosition(0);
  }

  @Override
  protected void execute() {
    Robot.m_climber.moveHand1(speed);
   // Robot.m_climber.moveHandsPosition(speed);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.m_climber.stopHand();
  }

  @Override
  protected void interrupted() {
  }
}
