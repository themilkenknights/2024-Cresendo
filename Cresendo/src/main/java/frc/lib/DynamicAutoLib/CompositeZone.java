
package frc.lib.DynamicAutoLib;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class for representing a zone for poses.
 * Uses Lists of Ellipse2ds and Rectangle2ds.
 */
public class CompositeZone {
    private ArrayList<Ellipse2d> ellipses = new ArrayList<>();
    private ArrayList<Rectangle2d> rectangles = new ArrayList<>();

    public CompositeZone(ArrayList<Ellipse2d> ellipses, ArrayList<Rectangle2d> rectangles) {
        this.ellipses = ellipses;
        this.rectangles = rectangles;
    }

    /**
     * @param pose To pose to check
     * @return wether or not the pose is in any of the zones
     */
    public boolean isInZone(Pose2d pose) {
        for (Ellipse2d ellipse2d : ellipses) {
            if (ellipse2d.contains(pose.getTranslation())) {
                return true;
            }
        }
        for (Rectangle2d rectangle2d : rectangles) {
            if (rectangle2d.contains(pose.getTranslation())) {
                return true;
            }
        }
        return false;
    }

    /**
     * @param pose To pose to check
     * @return wether or not the pose is in any of the zones
     */
    public boolean isInZone(Translation2d pose) {
        for (Ellipse2d ellipse2d : ellipses) {
            if (ellipse2d.contains(pose)) {
                return true;
            }
        }
        for (Rectangle2d rectangle2d : rectangles) {
            if (rectangle2d.contains(pose)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @param pose To pose to check
     * @return wether or not the pose is in any of the zones
     */
    public boolean isInZone(Transform2d pose) {
        for (Ellipse2d ellipse2d : ellipses) {
            if (ellipse2d.contains(pose.getTranslation())) {
                return true;
            }
        }
        for (Rectangle2d rectangle2d : rectangles) {
            if (rectangle2d.contains(pose.getTranslation())) {
                return true;
            }
        }
        return false;
    }

}
