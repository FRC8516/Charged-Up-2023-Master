// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedLights;
import frc.robot.Constants.OIConstants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CandleControl extends SubsystemBase {
   //Driver Station check for alliance to get color for the candle leds
   private Alliance alliance = Alliance.Invalid;
   private final int LEDS_PER_ANIMATION = 30;
    private final CANdle m_candle = new CANdle(OIConstants.CANdleID,"rio");
    private int m_candleChannel = 0;
    private boolean m_clearAllAnims = false;
    private boolean m_last5V = false;
    private boolean m_animDirection = false;
    private boolean m_setAnim = false;

    private Animation m_toAnimate = null;

    private enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Empty
    }

    private AnimationTypes m_currentAnimation;

  /** Creates a new CandleControl. */
  public CandleControl() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   //This checks the driver station for which alliance color we are
   public void checkDSUpdate() {
    //Read the current driver station alliance color
    Alliance currentAlliance = DriverStation.getAlliance();

    // If we have data, and have a new alliance from last time
    if (DriverStation.isDSAttached() && currentAlliance != alliance) {
      //call the subsystem for setting the led lights
      this.AllianceColor(currentAlliance);
    }
  } 

  //Init color from driver station connection
  private void AllianceColor(Alliance m_Alliance) {
    if (m_Alliance == Alliance.Red) {
      this.ChangeLedColor(LedLights.Red);
    }
    else {
      this.ChangeLedColor(LedLights.Blue);
    }
  }
  
  //change color to GREEN
  public void InTeleOpMode () {
    this.ChangeLedColor(LedLights.Green);
  }

  //Used to make generic calls to change lights
  public void ChangeLedColor(String mColor) {

    switch (mColor) {
      case LedLights.Yellow:;  //Nofity Human Player that drive team needs a cone.
        
        break;
      case LedLights.Purple:; //Nofity Human Player that drive team needs a cube.
        
        break;
      case LedLights.Blue:;

        break;
      case LedLights.Red:;

        break;
      case LedLights.Green:;

        break;
    }

  }

  private void incrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
        case Fire: changeAnimation(AnimationTypes.Larson); break;
        case Larson: changeAnimation(AnimationTypes.Rainbow); break;
        case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
        case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
        case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
        case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
        case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
        case TwinkleOff: changeAnimation(AnimationTypes.Empty); break;
        case Empty: changeAnimation(AnimationTypes.ColorFlow); break;
        case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    }
  }

  private void decrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: changeAnimation(AnimationTypes.Empty); break;
        case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
        case Larson: changeAnimation(AnimationTypes.Fire); break;
        case Rainbow: changeAnimation(AnimationTypes.Larson); break;
        case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
        case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
        case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
        case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
        case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
        case Empty: changeAnimation(AnimationTypes.TwinkleOff); break;
        case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    }
  }

 /* Wrappers so we can access the CANdle from the subsystem */
  private double getVbat() { return m_candle.getBusVoltage(); }
  private double get5V() { return m_candle.get5VRailVoltage(); }
  private double getCurrent() { return m_candle.getCurrent(); }
  private double getTemperature() { return m_candle.getTemperature(); }
  private void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
  private void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
  private void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
  private void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

 private void changeAnimation(AnimationTypes toChange) {
     m_currentAnimation = toChange;
     
     switch(toChange)
     {
         default:
         case ColorFlow:
             m_candleChannel = 0;
             m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_PER_ANIMATION, Direction.Forward, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case Fire:
             m_candleChannel = 1;
             m_toAnimate = new FireAnimation(0.5, 0.7, LEDS_PER_ANIMATION, 0.8, 0.5, m_animDirection, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case Larson:
             m_candleChannel = 2;
             m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 0.1, LEDS_PER_ANIMATION, BounceMode.Front, 3, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case Rainbow:
             m_candleChannel = 3;
             m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case RgbFade:
             m_candleChannel = 4;
             m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case SingleFade:
             m_candleChannel = 5;
             m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case Strobe:
             m_candleChannel = 6;
             m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case Twinkle:
             m_candleChannel = 7;
             m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_PER_ANIMATION, TwinklePercent.Percent42, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case TwinkleOff:
             m_candleChannel = 8;
             m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_PER_ANIMATION, TwinkleOffPercent.Percent76, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;
         case Empty:
             m_candleChannel = 9;
             m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection, m_candleChannel * LEDS_PER_ANIMATION + 8);
             break;

         case SetAll:
             m_toAnimate = null;
             break;
     }
     System.out.println("Changed to " + m_currentAnimation.toString());
 }

 public void clearAllAnims() {m_clearAllAnims = true;}

}
