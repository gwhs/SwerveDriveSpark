/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SwitchLedModes extends Command {

  boolean done;
  public SwitchLedModes() {
    requires(Robot.m_Limelight);
  }

  @Override
   protected void initialize() {
    done = false;
   }

  @Override
  protected void execute() {
    Robot.m_Limelight.switchModes();
    done = true;
   
  }

  @Override
  protected boolean isFinished() {
    return done;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
