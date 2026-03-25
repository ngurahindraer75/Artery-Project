package artery.application;

import omnetpp.cSimpleModule;

// Mendeklarasikan sebuah modul sederhana bernama CamLogger
simple CamLogger
{
    parameters:
        string logFileName = default("results/cam_log.csv");
    
    @class(CamLogger); 
}
