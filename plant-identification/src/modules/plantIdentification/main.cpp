#include "iCub/plantIdentification/PlantIdentificationModule.h" 

using yarp::os::Network;
using yarp::os::ResourceFinder;

YARP_DECLARE_DEVICES(icubmod);

int main(int argc, char * argv[])
{
    /*initializing device driver list */
    YARP_REGISTER_DEVICES(icubmod);
    
    /* initializing yarp network */ 
    Network yarp;
    if (!yarp.checkNetwork()) {
        std::cerr << "Error: yarp server is not available. \n";
        return -1;
    }

    iCub::plantIdentification::PlantIdentificationModule plantIdentification;

    /* preparing and configuring the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("confPlantIdentification.ini");
    rf.setDefaultContext("plantIdentification");
    rf.configure(argc, argv);

    /* starting module */
    plantIdentification.runModule(rf);

    return 0;
}
