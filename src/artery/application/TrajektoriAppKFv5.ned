package artery.application;

// Registering the TrajektoriAppKFv5 class to OMNeT++ simulation engine
simple TrajektoriAppKFv5 {
    parameters:
        double logInterval @unit(s) = default(0.1s);
        double staticSpeedThreshold = default(0.5);
                
        @display("i=block/app");
        @class(TrajektoriAppKFv5);
}