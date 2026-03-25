package artery.inet;

import artery.inet.World;
import artery.application.CamLogger;
import artery.application.TrajektoriApp;

network TrajektoriLogger extends World
{
    submodules:
        logger: CamLogger {
            parameters:
                @display("p=180,20;i=block/wr_s");
        }
}
