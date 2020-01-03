/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.subsystems;

import org.usfirst.frc.team5507.robot.commands.CargoIn;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Cargo extends Subsystem {
  DoubleSolenoid solenoid1 = new DoubleSolenoid(0, 1);
  private boolean isRetracted = false;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new CargoIn());
  }

  public void pushCargo() {
    solenoid1.set(DoubleSolenoid.Value.kForward);
  }

  public void pullCargo() {
    solenoid1.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isRetracted() {
    return isRetracted();
  }
}
