package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class TuningMathTest {

  @Test
  public void brakingFitRecoversKnownCoefficients() {
    double kLin = 0.0587;
    double kQuad = 0.00156;
    List<double[]> samples = new ArrayList<>();
    for (double v : new double[] {10, 20, 30, 45, 60, 75}) {
      samples.add(new double[] {v, kLin * v + kQuad * v * v});
    }
    double[] fit = BrakingTuner.fitBraking(samples);
    assertEquals(kLin, fit[0], 1e-9);
    assertEquals(kQuad, fit[1], 1e-9);
  }

  @Test
  public void brakingFitHasNoIntercept() {
    // Pure linear data through the origin must give zero quadratic term.
    List<double[]> samples = new ArrayList<>();
    for (double v : new double[] {5, 15, 25, 35}) {
      samples.add(new double[] {v, 0.1 * v});
    }
    double[] fit = BrakingTuner.fitBraking(samples);
    assertEquals(0.1, fit[0], 1e-9);
    assertEquals(0.0, fit[1], 1e-9);
  }

  @Test
  public void totalHeadingAccumulatesAcrossWrap() {
    TotalHeading total = new TotalHeading();
    double result = 0;
    // Two full counter-clockwise turns in 10-degree steps, reported wrapped to [0, 2pi).
    for (int deg = 0; deg <= 720; deg += 10) {
      double wrapped = Math.toRadians(deg % 360);
      result = total.update(wrapped);
    }
    assertEquals(4 * Math.PI, result, 1e-9);
  }

  @Test
  public void totalHeadingTracksClockwise() {
    TotalHeading total = new TotalHeading();
    double result = 0;
    for (int deg = 0; deg >= -180; deg -= 15) {
      double wrapped = Math.toRadians(((deg % 360) + 360) % 360);
      result = total.update(wrapped);
    }
    assertEquals(-Math.PI, result, 1e-9);
  }
}
