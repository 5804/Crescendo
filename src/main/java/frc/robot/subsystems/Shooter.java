// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFX rightShooterMotor;
  private TalonFX leftShooterMotor;
  private CANcoder shooterAngleEncoder;

  /** Creates a new Shooter. */
  public Shooter() {

    shooterAngleEncoder = new CANcoder(53);
    rightShooterMotor = new TalonFX(52);
    leftShooterMotor = new TalonFX(51);
    leftShooterMotor.setInverted(true);

  }

  public void shoot(double speed) {
    rightShooterMotor.set(speed);
    leftShooterMotor.set(speed);

  }

  public Command setShooterSpeedCommand() {
    return runOnce(
            () -> {
              shoot(0.5);
            })
        .withName("SetShooterSpeed");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
