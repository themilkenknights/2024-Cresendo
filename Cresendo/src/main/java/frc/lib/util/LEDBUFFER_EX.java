// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** An extention of the AddressableLEDBuffer class for easer use. */
public class LEDBUFFER_EX extends AddressableLEDBuffer {
  public LEDBUFFER_EX(int length) {
    super(length);
  }

  public void setSegmentHSV(int start, int end, int h, int s, int v) {
    for (int i = start; i < end; i++) {
      // Sets the specified LED to the RGB values for input
      this.setHSV(i, h, s, v);
    }
  }

  public void setSegmentRGB(int start, int end, int r, int g, int b) {
    for (int i = start; i < end; i++) {
      // Sets the specified LED to the RGB values for input
      this.setRGB(i, r, g, b);
    }
  }

  // TODO: add javadoc

  // TODO: add rainbow segmen
  // TODO: add animation options
  public void setSolidHSV(int h, int s, int v) {
    for (int i = 0; i < this.getLength(); i++) {
      // Sets the specified LED to the RGB values for input
      this.setHSV(i, h, s, v);
    }
  }

  public void setSolidRGB(int r, int g, int b) {
    for (int i = 0; i < this.getLength(); i++) {
      // Sets the specified LED to the RGB values for input
      this.setRGB(i, r, g, b);
    }
  }

  int rainbowStartHue;
  int rainbowSpeed;
  int rainbowSaturation;
  int rainbowValue;

  /**
   * @param startHue the current hue or hue to start at
   * @param s        saturation
   * @param v        value
   * 
   */
  public void setRainbow(int startHue, int s, int v) {
    rainbowSaturation = s;
    rainbowValue = v;
    rainbowStartHue = startHue;
    // For every pixel
    for (var i = 0; i < this.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (rainbowStartHue + (i * 180 / this.getLength())) % 180;
      // Set the value
      this.setHSV(i, hue, s, v);
    }
    // Check bounds
    rainbowStartHue %= 180;
  }

  /**
   * @param startHue the current hue or hue to start at
   * @param s        saturation
   * @param v        value
   * @param maxHue   maximum befrofe modular
   */
  public void setRainbow(int startHue, int s, int v, int speed, int maxHue) {
    rainbowSaturation = s;
    rainbowValue = v;
    rainbowStartHue = startHue;
    for (var i = 0; i < this.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (rainbowStartHue + (i * maxHue / this.getLength())) % maxHue;
      // Set the value
      this.setHSV(i, hue, s, v);
    }
    // Check bounds
    rainbowStartHue %= maxHue;
    // // Increase by to make the rainbow "move"
    // startHue += speed;
  }

  public void UpdateRainbow() {
    rainbowSpeed += rainbowSpeed;
    setRainbow(rainbowStartHue, rainbowSaturation, rainbowValue);
  }

}
