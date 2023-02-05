package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
  }

  public void setLEDsBlue() {
    for(var i = 0; i < getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 255);
    }
  }

  public void setLEDsGreen() {
    for(var i = 0; i < getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public void setLEDsTargetSeen() {
    if(targetSeen && !targetAligned) {
      setBottomLEDs(255, 165, 0); // orange
    } else {
      setBottomLEDs(0, 0, 0); // black
    }
  }

  public void setLEDsTargetAligned() {
    if(targetAligned && targetSeen) {
      setBottomLEDs(0, 255, 0); // green
    } else {
      setBottomLEDs(0, 0, 0); // black
    }
  }

  public void setLEDsConeMode() {
    if(coneMode && !cubeMode) {
      setMiddleLEDs(255, 255, 0); // yellow
    } else {
      setMiddleLEDs(0, 0, 0); // black
    }
  }

  public void setLEDsCubeMode() {
    if(cubeMode && !coneMode) {
      setMiddleLEDs(160, 32, 240); // purple
    } else {
      setMiddleLEDs(0, 0, 0); // black
    }
  }

  public void setLEDsCubeLoading() {
    if(cubeLoading && !cubeLoaded && !objectHeld) {
      if(cubeMode) {
        setTopLEDs(101, 67, 33); // dark brown
      } else {
        cubeMode = true;
      }
    } else {
      setTopLEDs(0, 0, 0); // black
    }
  }

  public void setLEDsCubeLoaded() {
    if(cubeLoaded && !cubeLoading && !objectHeld) {
      if(cubeMode) {
        setTopLEDs(255, 165, 0); // orange
      } else {
        cubeMode = true;
      }
    } else {
      setTopLEDs(0, 0, 0); // black
    }
  }

  public void setLEDsObjectHeld() {
    if(objectHeld) {
      setTopLEDs(0, 255, 0); // green
    } else {
      setTopLEDs(0, 0, 0); // black
    }
  }

  public void setBottomLEDs(int r, int g, int b) {
    for(var i = 0; i < getLength(); i++) {
      if(i < (getLength() / 3)) {
          ledBuffer.setRGB(i, r, g, b);
      } else {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void setMiddleLEDs(int r, int g, int b) {
    for(var i = 0; i < getLength(); i++) {
      if(i > (getLength() / 3) && i < 2 * (getLength() / 3)) {
        ledBuffer.setRGB(i, r, g, b);
      } else {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void setTopLEDs(int r, int g, int b) {
    for(var i = 0; i < getLength(); i++) {
      if(i > 2 * (getLength() / 3)) {
        ledBuffer.setRGB(i, r, g, b);
      } else {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
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