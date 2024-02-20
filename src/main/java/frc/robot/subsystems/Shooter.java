// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {

  private TalonFX rightShooterMotor;
  private TalonFX leftShooterMotor;
  private TalonFX indexerMotor;
  private TalonFX rightShooterAngleMotor;
  private TalonFX leftShooterAngleMotor;
  private CANcoder shooterAngleEncoder;
  private Rotation2d angleOffset;

 

  /** Creates a new Shooter. */
  public Shooter() {

    shooterAngleEncoder = new CANcoder(53);
    rightShooterMotor = new TalonFX(52);
    leftShooterMotor = new TalonFX(51);
    indexerMotor = new TalonFX(54);
    rightShooterAngleMotor = new TalonFX(61);
    leftShooterAngleMotor = new TalonFX(60);
    leftShooterMotor.setInverted(true);
    
    rightShooterAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    leftShooterAngleMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  //not sure if we need this
 public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(shooterAngleEncoder.getAbsolutePosition().getValue());
    }
  //and this
    public void resetToAbsolute(){
      double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();

  }

  public void shoot(double shootSpeed) {
    rightShooterMotor.set(shootSpeed);
    leftShooterMotor.set(shootSpeed);
    //intakeShooterMotor.set(speed);

  }

  public void load(double indexerSpeed) {
    indexerMotor.set(indexerSpeed);
  }

  boolean enableShooter = true;
  public Command setShooterSpeedCommand(double shooterSpeed) {
    return runOnce(
            () -> {
              if (enableShooter == true) {
                shoot(shooterSpeed); 
                enableShooter = false;
              } else {
                shoot(0);
                enableShooter = true;
              }
            })
        .withName("SetShooterSpeed");
  }
/*
  public Command enableShooter() {
    return runOnce(
            () -> {
              shoot(shooterSpeed);
            })
        .withName("DisableShooter");
  }

  public Command disableShooter() {
    return runOnce(
            () -> {
              shoot(0);
            })
        .withName("DisableShooter");
  }
*/
  boolean indexerEnable = true;
  public Command setIndexerSpeedCommand() {
    return runOnce(
            () -> {
              if (indexerEnable == true) {
                load(0.8); 
                indexerEnable = false;
              } else {
                load(0);
                indexerEnable = true;
              }
            })
        .withName("Load");
  }

  public Command raiseShooter() {
    return runOnce(
      () -> {
        angleShooter();
      }
    );
  }

  public Command lowerShooter() {
    return runOnce(
      () -> {
        angleShooter();
      }
    );
  }

  public Command angleShooter() {
    return run(
      () -> { 
        //rightShooterAngleMotor.set(1);
        //leftShooterAngleMotor.set(1);
      }
    );
  }
/* // Shooter speed adjust buttons
  public Command increaseShooterSpeed() { // Increase and decrease speed commands are temporary features for debugging
    return runOnce(
      () -> {
        if (shooterSpeed < 1) {
          shooterSpeed += .1;
          System.out.println("Current shooter speed: " + shooterSpeed);
          if (enableShooter == false) {
            shoot(shooterSpeed);
          }
        }
      })
      .withName("IncreaseShooterSpeed");
  }

  public Command decreaseShooterSpeed() {
    return runOnce(
      () -> {
        if (shooterSpeed > 0) {
          shooterSpeed -= .1;
          System.out.println("Current shooter speed: " + shooterSpeed);
          if (enableShooter == false) {
            shoot(shooterSpeed);
          }
        }
      })
      .withName("DecreaseShooterSpeed");
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Rotation2d rotations = Rotation2d.fromRotations(shooterAngleEncoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("ShooterAngle", rotations.getDegrees());
  }
}
