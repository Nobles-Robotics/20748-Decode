package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

// NOTE: Pedro headings are radians. :contentReference[oaicite:2]{index=2}
public class WiggleUtil {

    /**
     * @param baseHeading heading center (radians)
     * @param amplitude   max wiggle left/right (radians), e.g. Math.toRadians(10)
     * @param cycles      how many left-right cycles over the whole path
     * @param segments    more segments = smoother (try 24–60)
     */
    public static HeadingInterpolator makeWiggle(
            double baseHeading, double amplitude, int cycles, int segments
    ) {
        HeadingInterpolator.PiecewiseNode[] nodes = new HeadingInterpolator.PiecewiseNode[segments];

        double prevT = 0.0;
        double prevHeading = baseHeading + amplitude * Math.sin(0.0);

        for (int i = 0; i < segments; i++) {
            double t0 = (double) i / segments;
            double t1 = (double) (i + 1) / segments;

            double phase1 = 2.0 * Math.PI * cycles * t1;
            double h1 = baseHeading + amplitude * Math.sin(phase1);

            nodes[i] = new HeadingInterpolator.PiecewiseNode(
                    t0,
                    t1,
                    HeadingInterpolator.linear(prevHeading, h1)
            );

            prevT = t1;
            prevHeading = h1;
        }

        return HeadingInterpolator.piecewise(nodes);
    }

    public static Path makeStraightWigglePath(Pose start, Pose end) {
        Path p = new Path(new BezierLine(start, end)); // straight line
        p.setHeadingInterpolation(
                makeWiggle(
                        start.getHeading(),          // center heading
                        Math.toRadians(15),          // amplitude (try 5–15°)
                        3,                           // cycles along the path
                        48                           // smoothness
                )
        );
        return p;
    }
}