/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import org.usfirst.frc.team5507.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberMoveArms extends Command {

  public ClimberMoveArms() {
    requires(Robot.m_climber);
  }

  @Override
  protected void initialize() {
    Robot.m_climber.getHand().setIdleMode(IdleMode.kBrake);
  }

  private double deadband(double input) {
		if (Math.abs(input) < 0.15) return 0;
		return input;
  }
  
  @Override
  protected void execute() {
    double speed1 = deadband(Robot.getOI().getClimberController().getLeftYValue());
    double speed2 = deadband(Robot.getOI().getClimberController().getRightYValue());
    Robot.m_climber.moveArms(speed1, speed2);
    if(speed1 < 0) Robot.m_climber.moveHand1(0.25);
    else {
      Robot.m_climber.moveHand1(0);
    }

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
