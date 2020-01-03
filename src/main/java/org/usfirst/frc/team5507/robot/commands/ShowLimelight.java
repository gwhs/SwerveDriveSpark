/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;


import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team5507.robot.Robot;

public class ShowLimelight extends Command {
  public ShowLimelight() {
    requires(Robot.m_Limelight);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.m_Limelight.printInfo();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
