package artery.inet;

import artery.inet.World;
import artery.application.CamLogger;

network Logger extends World
{
    submodules:
        logger: CamLogger {
        	parameters:
            	@display("p=180,20;i=block/wr_s");
    	}

}
