package artery.application;

simple TrajAppRL
{
    parameters:
        double logInterval @unit(s) = default(0.1s);
        double staticSpeedThreshold = default(0.5);
        
        // --- NEW PARAMETER: DYNAMIC PREDICTION HORIZON ---
        double predictionHorizon @unit(s) = default(1.0s); 
        
        @display("i=block/app");
        @class(TrajAppRL);
}