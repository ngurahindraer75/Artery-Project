package artery.application;

// Registering the TrajAppKF4D class to OMNeT++ simulation engine
simple TrajAppKF4D {
    parameters:
        double logInterval @unit(s) = default(0.1s);
        double staticSpeedThreshold = default(0.5);
                
        @display("i=block/app");
        @class(TrajAppKF4D);
}