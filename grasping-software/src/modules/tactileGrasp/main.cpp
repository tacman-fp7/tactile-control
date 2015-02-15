
/* 
 * Copyright (C) 2015 Massimo Regoli, iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Francesco Giovannini, Massimo Regoli
 * email:   francesco.giovannini@iit.it, massimo.regoli@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */



#include "iCub/tactileGrasp/TactileGraspModule.h" 

using yarp::os::Network;
using yarp::os::ResourceFinder;


YARP_DECLARE_DEVICES(icubmod);

int main(int argc, char * argv[])
{
    // Initialise device driver list
    YARP_REGISTER_DEVICES(icubmod);
    
    /* initialize yarp network */ 
    Network yarp;
    if (!yarp.checkNetwork()) {
        std::cerr << "Error: yarp server is not available. \n";
        return -1;
    }     

    /* create your tactileGrasp */
    iCub::tactileGrasp::TactileGraspModule tactileGrasp;

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("confTactileGrasp.ini");
    rf.setDefaultContext("tactileGrasp");
    rf.configure(argc, argv);

    /* run the tactileGrasp: runModule() calls configure first and, if successful, it then runs */
    tactileGrasp.runModule(rf);

    return 0;
}
