package artery.application;

simple TrajektoriAppKFv3
{
    parameters:
        double logInterval @unit(s) = default(0.1s);
        double staticSpeedThreshold = default(0.5);
        
        // --- PARAMETER HORIZON DINAMIS ---
        double predictionHorizon @unit(s) = default(1.0s); 
        
        @display("i=block/app");
        @class(TrajektoriAppKFv3);
}