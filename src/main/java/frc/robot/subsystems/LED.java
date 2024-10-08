package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private static final CANdle candle1 = new CANdle(26);
  private static final CANdle candle2 = new CANdle(27);

  public void CANdleSystem() {
    CANdleConfiguration configAll = new CANdleConfiguration();

    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.5;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle1.configAllSettings(configAll, 100);
    candle2.configAllSettings(configAll, 100);
  }

  public static void LEDColor(int r, int g, int b) {
    candle1.animate(null);
    candle2.animate(null);
    candle1.setLEDs(r, g, b);
    candle2.setLEDs(r, g, b);
  }

  // public static void Off() {
  //   candle1.setLEDs(0, 0, 0);
  //   candle2.setLEDs(0, 0, 0);
  // }

  // public Command lightsOff() {
  //   return runOnce(
  //       () -> {
  //           Off();
  //       }
  //   );
  // }

  public Command Off() {
    return runOnce(
        () -> {
          LEDColor(0,0,0);
        }
    );
  }

  public Command Green() {
    return runOnce(
      () -> {
        LEDColor(0, 255, 0);
      }
    );
  }

  public Command Red() {
    return runOnce(
      () -> {
        LEDColor(255, 0, 0);
      }
    );
  }

  // public static void Rainbow(int num) {
  //   RainbowAnimation rainbowAnim = new RainbowAnimation(1, .5, num);
  //   candle1.animate(rainbowAnim);
  //   candle2.animate(rainbowAnim);
  // }

  // public static void Rainbow() {
  //   RainbowAnimation rainbowAnim = new RainbowAnimation(1, .5, 100);
  //   candle1.animate(rainbowAnim);
  //   candle2.animate(rainbowAnim);
  // }

  public static void StrobeGreen() {
    StrobeAnimation strobe = new StrobeAnimation(0, 255, 0);
    candle1.animate(strobe);
    candle2.animate(strobe);
  }

  public static void Twinkle() {
    TwinkleAnimation twinkle = new TwinkleAnimation(
      0,
      255,
      0,
      0,
      .5,
      100,
      TwinklePercent.Percent100
    );
    candle1.animate(twinkle);
    candle2.animate(twinkle);
  }


  public static void YellowFlow() {
    ColorFlowAnimation colorflow = new ColorFlowAnimation(255, 255, 0);
    candle1.animate(colorflow);
    candle2.animate(colorflow);
  }

  public Command setYellowCommand() {
    return runOnce(
        () -> {
            YellowFlow();
        }
    );
  }

  // public Command setRainbowCommand() {
  //   return runOnce(
  //       () -> {
  //           Rainbow();
  //       }
  //   );
  // }

  public static void GreenFlow() {
    ColorFlowAnimation colorflow = new ColorFlowAnimation(0, 255, 0);
    candle1.animate(colorflow);
    candle2.animate(colorflow);
  }

   public static void redFlow() {
    ColorFlowAnimation colorflow = new ColorFlowAnimation(225, 0, 0);
    candle1.animate(colorflow);
    candle2.animate(colorflow);
  }

  // public static void orange(){
  //   candle1.setLEDs(255, 150, 0);
  // }

  // public static void red(){
  //   candle1.setLEDs(255, 0, 0);
  // }

  //  public static void yellow(){
  //   candle1.setLEDs(255, 255, 0);
  // }

  // public Command setOrangeCommand(){
  //   return runOnce( 
  //     () -> {
  //       orange();
  //     }
  //   );
  // }

  public Command setGreenCommand() {
    return runOnce(
        () -> {
            GreenFlow();
        }
    );
  }

  public Command setRedCommand() {
    return runOnce(
        () -> {
            redFlow();
        }
    );
  }

  public static void Fire() {
    FireAnimation fireAnim = new FireAnimation(1, 1, 15, 1, 1, false, 0);
    candle1.animate(fireAnim);
    candle2.animate(fireAnim);
  }

  public Command SetAnimationFire() {
    return runOnce(
            () -> {
              Fire();
            })
        .withName("SetFireAnimation");
  }

  public boolean greenDebounce = true;

  // @Override
  // public void periodic() {
  //   if (!DriverStation.getJoystickIsXbox(0)) {
  //     DriverStation.reportError("Rescan!!!!", false);
  //     candle1.animate(null);
  //     candle2.animate(null);
  //     candle1.setLEDs(255, 0, 0);
  //     candle2.setLEDs(255, 0, 0);
  //     greenDebounce = false;
  //   } else {
  //     if (greenDebounce == false) {
  //       RainbowAnimation rainbowAnim = new RainbowAnimation(1, .5, 100);
  //       candle1.animate(rainbowAnim);
  //       candle2.animate(rainbowAnim);
  //       greenDebounce = true;
  //     }
  //   }
  // }
}
