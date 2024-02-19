package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor;
    private TalonFX angleMotor;
    private CANcoder angleEncoder;

    public Intake() {
        intakeMotor = new TalonFX(56);
        intakeMotor.setInverted(true);
        angleMotor = new TalonFX(59);
        angleMotor.setInverted(true);
        angleMotor.setNeutralMode(NeutralModeValue.Coast);
        angleEncoder = new CANcoder(54);



        /* Set relevant frame periods to be at least as fast as periodic rate */
        // angleMotor.setStatusFramePeriod(
        // StatusFrameEnhanced.Status_13_Base_PIDF0,
        // 10,
        // Constants.Intake.kTimeoutMs
        // );

        // angleMotor.setStatusFramePeriod(
        // StatusFrameEnhanced.Status_10_MotionMagic,
        // 10,
        // Constants.Intake.kTimeoutMs
        // );

        // /* Set the peak and nominal outputs */
        // angleMotor.configPeakOutputForward(1, Constants.Intake.kTimeoutMs);
        // angleMotor.configPeakOutputReverse(-1, Constants.Intake.kTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kV = 0.12;
        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0.5;
        slot0Configs.kD = 0.001;
        slot0Configs.kS = 0;

        angleMotor.getConfigurator().apply(slot0Configs, 0.050);

        var motionMagicConfigs = talonFXConfigs.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;

        angleMotor.getConfigurator().apply(talonFXConfigs, 0.050);

        // angleMotor.config_IntegralZone(Constants.Intake.kSlotIdx0, 200);
        // angleMotor.configAllowableClosedloopError(Constants.Intake.kSlotIdx0, 400);

        // /* Set Motion Magic gains in slot1 - see documentation */
        // angleMotor.selectProfileSlot(Constants.Intake.kSlotIdx1, Constants.Intake.kPIDLoopIdx);
        // angleMotor.config_kF(Constants.Intake.kSlotIdx1, 0.0714, Constants.Intake.kTimeoutMs);
        // angleMotor.config_kP(Constants.Intake.kSlotIdx1, 0.1, Constants.Intake.kTimeoutMs);
        // angleMotor.config_kI(Constants.Intake.kSlotIdx1, 0, Constants.Intake.kTimeoutMs);
        // angleMotor.config_kD(Constants.Intake.kSlotIdx1, 0, Constants.Intake.kTimeoutMs);
        // angleMotor.config_IntegralZone(Constants.Intake.kSlotIdx1, 200);
        // angleMotor.configAllowableClosedloopError(Constants.Intake.kSlotIdx1, 100);

        // /* Set acceleration and vcruise velocity - see documentation */
        // angleMotor.configMotionCruiseVelocity(14000, Constants.Intake.kTimeoutMs);
        // angleMotor.configMotionAcceleration(23000, Constants.Intake.kTimeoutMs);
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
