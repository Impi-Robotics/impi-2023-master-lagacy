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
  private boolean targetAligned;
  private boolean targetSeen;
  private boolean cubeLoading;
  private boolean cubeLoaded;
  private boolean objectHeld;

  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED.PORT);
    ledBuffer = new AddressableLEDBuffer(Constants.LED.LENGTH);

    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLEDBuffer() {
    led.setData(ledBuffer);
  }

  public void setLEDsToColor(int r, int g, int b) {
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

  public void setMiddleLEDs(int r, int g, int b) {
    for(var i = 0; i < getLength(); i++) {
      if(i > (getLength() / 3) && i < 2 * (getLength() / 3)) {
        ledBuffer.setRGB(i, r, g, b);
      } else {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(ledBuffer);
  }

  public void setTopLEDs(int r, int g, int b) {
    for(var i = 0; i < getLength(); i++) {
      if(i > 2 * (getLength() / 3)) {
        ledBuffer.setRGB(i, r, g, b);
      } else {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
      led.setData(ledBuffer);
    }
  }

  // public void coneMode(boolean coneMode) {
  //   this.coneMode = coneMode;
  // }

  // public void cubeMode(boolean cubeMode) {
  //   this.cubeMode = cubeMode;
  // }

  public void setCubeMode() {
    coneMode = false;
    cubeMode = true;
  }

  public void coneMode(boolean coneMode) {
    this.coneMode = coneMode;
  }

  public void setConeMode() {
    cubeMode = false;
    coneMode = true;
  }

  public void setTargetSeen() {
    targetSeen = true;
    targetAligned = false;
  }

  public void setTargetAligned() {
    targetSeen = true;
    targetAligned = true;
  }

  public void setCubeLoading() {
    cubeLoaded = false;
    cubeLoading = true;
  }

  public void setCubeLoaded() {
    cubeLoading = false;
    cubeLoaded = true;
  }

  public void setObjectHeld() {
    objectHeld = true;
  }

  public void setObjectVacant() {
    objectHeld = false;
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
}