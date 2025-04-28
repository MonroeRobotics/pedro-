package MonroeRobotics.auto;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;


public class pedroAuto {
    static final Pose Basket = new Pose(68,48,Math.toRadians(225));
    static final Pose basketBackoff = new Pose();//I'll get cords latter
    static final Pose NeutSample1App = new Pose(38,58, Math.toRadians(-90));
    static final Pose NeutSample1 = new Pose(38,40,Math.toRadians(-90));
    static final Pose NeutSample2App = new Pose(64,58,Math.toRadians(-90));
    static final Pose NeutSample2 = new Pose(64,40,Math.toRadians(-90));
    static final Pose NeutSample3App = new Pose(64,62,Math.toRadians(-75));
    static final Pose NeutSample3 = new Pose(64,40,Math.toRadians(-75));
    static final Pose park = new Pose(45, 40,Math.toRadians(-90));
    enum autoState{
        start,
        sub,
        toNeutral,
        bucket,
        drop,
        bucketLeave,
        park,
        stop
    }
    autoState queuedState = autoState.start;

    public void buildPaths(){

    }

}

