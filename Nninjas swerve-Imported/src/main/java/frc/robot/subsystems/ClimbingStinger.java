// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingStinger extends SubsystemBase {
WPI_VictorSPX _slave;
WPI_TalonSRX _master;
  public ClimbingStinger() {
    _slave = new WPI_VictorSPX(0);
    _master = new WPI_TalonSRX(1);
    _slave.follow(_master);

    _master.setSensorPhase(false);

  }

  public void getToPoint(double _newTarget){
    _master.set(ControlMode.MotionMagic,_newTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
