/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class HatchPushBack extends Command {

  public HatchPushBack() {
    requires(Robot.swerveDriveSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.swerveDriveSubsystem.setFieldOriented(false);
  }

  @Override
  protected void execute() {
    Robot.swerveDriveSubsystem.holonomicDrive(-0.2, 0, 0);
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
    Robot.swerveDriveSubsystem.setFieldOriented(true);
  }

  @Override
  protected void interrupted() {
  }
}
