// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  private AddressableLED led;
  private static AddressableLEDBuffer ledBuffer;
  private boolean cubeMode;
  private boolean coneMode;

  public LEDSubsystem() {

    led = new AddressableLED(Constants.LED.PORT);
    ledBuffer = new AddressableLEDBuffer(Constants.LED.LENGTH);

    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void SetLEDBuffer() {
    led.setData(ledBuffer);
  }

  public void SetLEDsToColor(int r, int g, int b) {
    for(var i = 0; i < getLength(); i++) {
        ledBuffer.setRGB(i, r, g, b);
    }
    // FIXME
    // led.setData(ledBuffer);
    // setLEDs("Purple");
    // Timer timer;
    // timer.reset();
    // timer.start();
    // if(timer < 3){
    //   setLEDs("Green");
    // }
    // else{
    //   setLEDs("Purple");
    // }
    // }
  }

  /*
   * public void setLEDs(string color){
   *  switch(color){
   *    case "Purple":
   *        for(var i = 0; i < getLength(); i++) {
              ledBuffer.setRGB(i, 160,  30, 240);//Purple
            }
            led.setData(ledBuffer);
          }
   * }
   */
  //Cube LEDs
  public void SetLEDsCubeModeNotHolding() {
    for(var i = 0; i < getLength(); i++) {
      ledBuffer.setRGB(i, 160,  30, 240);//Purple
    }
    led.setData(ledBuffer);
  }

  public void SetLEDsCubeModeHolding() {
    for(var i = 0; i < getLength(); i++) {
      ledBuffer.setRGB(i, 0,  0, 255);//Blue
    }
    led.setData(ledBuffer);
  }


  //Cone LEDs
  public void SetLEDsConeModeNotHolding() {
    for(var i = 0; i < getLength(); i++) {
      ledBuffer.setRGB(i, 100,  64, 0);//Orange
    }
    led.setData(ledBuffer);
  }

  public void SetLEDsConeModeHolding() {
    for(var i = 0; i < getLength(); i++) {
      ledBuffer.setRGB(i, 100,  100, 0);//Yellow
    }
    led.setData(ledBuffer);
  }

  //Aligned LEDs
  public void SetLEDsAligned() {
    for(var i = 0; i < getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);//Green
    }
    led.setData(ledBuffer);
  }

  public void SetLEDsNotAligned() {
    for(var i = 0; i < getLength(); i++) {
      ledBuffer.setRGB(i, 255,  0, 0);//Red
    }
    led.setData(ledBuffer);
  }

  

  

  
//OLD
  public void LowPosition(int r, int g, int b) {
    for(var i = 0; i < getLength(); i++) {
      if(i < (getLength() / 3)) {
        ledBuffer.setRGB(i, r, g, b);
      }
    }
    led.setData(ledBuffer);
  }

  public void MediumPosition() {
    for(var i = 0; i < getLength(); i++) {
      if(i > (getLength() / 3) && i < 2 * (getLength() / 3)) {
        if(getConeMode()) {
          ledBuffer.setRGB(i, 255, 255, 0);
        } else if(getCubeMode()) {
          ledBuffer.setRGB(i, 160, 32, 240);
        }
      } else {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(ledBuffer);
  }

  public void HighPosition() {
    for(var i = 0; i < getLength(); i++) {
      if(i > 2 * (getLength() / 3)) {
        if(getConeMode()) {
          ledBuffer.setRGB(i, 255, 255, 0);
        } else if(getCubeMode()) {
          ledBuffer.setRGB(i, 160, 32, 240);
        }
      } else {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
      led.setData(ledBuffer);
    }
  }
  public void ConeMode(boolean coneMode) {
    this.coneMode = coneMode;
  }

  public void CubeMode(boolean cubeMode) {
    this.cubeMode = cubeMode;
  }

  public boolean getConeMode() {
    return coneMode;
  }

  public boolean getCubeMode() {
    return cubeMode;
  }

  public int getLength() {
    return ledBuffer.getLength();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}