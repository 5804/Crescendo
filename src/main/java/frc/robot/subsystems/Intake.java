package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public TalonFX intakeMotor;
    public TalonFX angleMotor;
    public CANcoder angleEncoder;
    //DutyCycleOut angleMotorRequest = new DutyCycleOut(0.0);
    //DigitalInput angleMotorForwardLimit = new DigitalInput(0);
    

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    
    public Intake() {
        intakeMotor = new TalonFX(56);
        intakeMotor.setInverted(true);
        angleMotor = new TalonFX(59);
        // angleMotor.setPosition(0);
        angleEncoder = new CANcoder(54);

        /* Set Motion Magic gains in slot0 - see documentation */

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kV = 7; // 7 
        slot0Configs.kP = 80; // 50 // 70 
        slot0Configs.kI = 0; // 0 
        slot0Configs.kD = 2; // 3.375 // 2 
        slot0Configs.kS = 0; // 0 

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 3; // 3
        motionMagicConfigs.MotionMagicAcceleration = 2.75; // 1.5 /// 2.75
        motionMagicConfigs.MotionMagicJerk = 0; // 0

        angleMotor.getConfigurator().apply(talonFXConfigs, 0.050);
        angleMotor.setInverted(false);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void load(double speed) {
        intakeMotor.set(speed);
    }

    public Command setIntakeSpeed(double intakeSpeed) {
        return run(
          () -> {
            load(intakeSpeed);
          }
        );
      }

    double intakeSpeed = 0.8;
    boolean enableIntake = true;
    public Command setIntakeSpeedCommand() {
        return runOnce(
                () -> {
                if (angleEncoder.getAbsolutePosition().getValue() < 0.145) {
                    intakeSpeed = 0;
                } else {
                    intakeSpeed = 0.8;
                }
                if (enableIntake == true) {
                    load(intakeSpeed);
                    enableIntake = false;
                } else {
                    load(0);
                    enableIntake = true;
                }
                })
            .withName("SetIntakeSpeed");
    }

    /*boolean enableAngle = true;
    public Command raiseIntake() {
        return runOnce(
          () -> {
            if (enableAngle == true) {
                setAngleSpeed();
                enableAngle = false;
            } else {
                setAngleSpeed();
                enableAngle = true;
            }
          })
        .withName("");
      }
    */

      public void setAnglePosition(double anglePosition) {
        MotionMagicVoltage request = new MotionMagicVoltage(0);

        angleMotor.setControl(request.withPosition(anglePosition));
      } // doesnt use encoder

      public Command stow() {
        return run(
            () -> {
                setAnglePosition(0.0); // 0.0175
            }
        ).finallyDo(
            () -> {
                setAngleSpeed(0);
            }
        );
      }

      public Command deploy() {
        return run(
            () -> {
                setAnglePosition(0.35);
            }
        ).finallyDo(
            () -> {
                setAngleSpeed(0);
            }
        );
      }

    
      public void setAngleSpeed(double angleSpeed) {
        angleMotor.set(angleSpeed);
        //angleMotorRequest.Output = angleSpeed;
        //angleMotor.setControl(angleMotorRequest.withOutput(angleSpeed).withLimitForwardMotion(angleMotorForwardLimit));
      }

    public Command lowerIntake() {
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

    public Command raiseIntake() {
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

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Rotation2d rotations = Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
        Rotation2d motorRotations = Rotation2d.fromRotations(angleMotor.getPosition().getValue());
        double motorVelocity = angleMotor.getVelocity().getValue();
        SmartDashboard.putNumber("IntakeAngle", rotations.getDegrees());
        SmartDashboard.putNumber("IntakeAngleEncoder", motorRotations.getDegrees());
        SmartDashboard.putNumber("MotorVelocity", motorVelocity);
    }
}
