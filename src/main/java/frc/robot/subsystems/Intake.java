package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor;
    private TalonFX angleMotor;
    private CANcoder angleEncoder;

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
        angleMotor.setInverted(false);
        angleMotor.setNeutralMode(NeutralModeValue.Coast);
        angleMotor.setPosition(0);
        angleEncoder = new CANcoder(54);

        /* Set Motion Magic gains in slot0 - see documentation */

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kV = 0.12;
        slot0Configs.kP = 4.8;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.1;
        slot0Configs.kS = 0.25;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.2;
        motionMagicConfigs.MotionMagicAcceleration = 0.1;
        motionMagicConfigs.MotionMagicJerk = 0;

        angleMotor.getConfigurator().apply(talonFXConfigs, 0.050);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    double intakeSpeed = 0.8;
    boolean enableIntake = true;
    public Command setIntakeSpeedCommand() {
        return runOnce(
                () -> {
                if (enableIntake == true) {
                    setIntakeSpeed(intakeSpeed);
                    enableIntake = false;
                } else {
                    setIntakeSpeed(0);
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
                setAnglePosition(300);
            }
        ).finallyDo(
            () -> {
                setAngleSpeed(0);
            }
        );
      }

    
      public void setAngleSpeed(double angleSpeed) {
        angleMotor.set(angleSpeed);
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
