// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

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
  private CANcoder angleEncoder;
  private Rotation2d angleOffset;
  public TimeOfFlight TOF;

  /** Creates a new Shooter. */
  public Shooter() {

    angleEncoder = new CANcoder(53);
    rightShooterMotor = new TalonFX(52);
    leftShooterMotor = new TalonFX(51);
    indexerMotor = new TalonFX(54);
    rightShooterAngleMotor = new TalonFX(61);
    leftShooterAngleMotor = new TalonFX(60);
    //rightShooterAngleMotor.setInverted(false);
    //leftShooterAngleMotor.setInverted(true);
    leftShooterMotor.setInverted(true);

    TOF = new TimeOfFlight(1);
    TOF.setRangingMode(RangingMode.Short, 30);
    
    rightShooterAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    leftShooterAngleMotor.setNeutralMode(NeutralModeValue.Brake);

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kV = 3;
    slot0Configs.kP = 10;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kS = 0;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 1;
    motionMagicConfigs.MotionMagicAcceleration = 1;
    motionMagicConfigs.MotionMagicJerk = 0;

    leftShooterAngleMotor.getConfigurator().apply(talonFXConfigs, 0.050);
    rightShooterAngleMotor.getConfigurator().apply(talonFXConfigs, 0.050);
    rightShooterAngleMotor.setControl(new Follower(leftShooterAngleMotor.getDeviceID(), true));

  }

  //not sure if we need this
 public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
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


  public void setAngleSpeed(double angleSpeed) {
    leftShooterAngleMotor.set(angleSpeed);
  }

  public Command lowerShooter() {
    return run(
        () -> {
            setAngleSpeed(-0.1);
        }
    ).finallyDo(
        () -> {
            setAngleSpeed(0);
        }
    );
}

public Command raiseShooter() {
    return run(
        () -> {
            setAngleSpeed(0.1);
        }
    ).finallyDo(
        () -> {
            setAngleSpeed(0);
        }
    );
}

  public void setAnglePosition(double anglePosition) {
        MotionMagicVoltage request = new MotionMagicVoltage(0);

        leftShooterAngleMotor.setControl(request.withPosition(anglePosition));
      } // doesnt use encoder

      public Command stow() {
        return run(
            () -> {
                setAnglePosition(0.011);
            }
        ).finallyDo(
            () -> {
                setAngleSpeed(0);
            }
        );
      }

      public Command amp() {
        return run(
            () -> {
                setAnglePosition(0.1375);
            }
        ).finallyDo(
            () -> {
                setAngleSpeed(0);
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
    Rotation2d rotations = Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    Rotation2d motorRotations = Rotation2d.fromRotations(leftShooterAngleMotor.getPosition().getValue());
    SmartDashboard.putNumber("ShooterAngle", rotations.getDegrees());
    double motorVelocity = leftShooterAngleMotor.getVelocity().getValue();
    SmartDashboard.putNumber("ShooterIntakeAngle", rotations.getDegrees());
    SmartDashboard.putNumber("ShooterIntakeAngleEncoder", motorRotations.getDegrees());
    SmartDashboard.putNumber("ShooterMotorVelocity", motorVelocity);
    SmartDashboard.putNumber("TimeOfFlightSensor", TOF.getRange());
  }
}
