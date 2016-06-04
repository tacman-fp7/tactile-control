#include "iCub/plantIdentification/data/GMMData.h"

#include "iCub/plantIdentification/util/ICubUtil.h"

#include <yarp/math/Math.h>
#include <math.h>

using iCub::plantIdentification::GMMData;

using yarp::os::Bottle;


GMMData::GMMData() {
	using iCub::plantIdentification::ICubUtil;

	numComponents = 2;
	int numDimensions = 8;

	mu.resize(numComponents);
	muQ.resize(numComponents);
	muR.resize(numComponents);
	sigma.resize(numComponents);
	sigmaQQ.resize(numComponents);
	sigmaQQInv.resize(numComponents);
	sigmaQQDet.resize(numComponents);
	sigmaRQ.resize(numComponents);
	sigmaRR.resize(numComponents);
	componentsPrior.resize(numComponents);
	

	double muArray_0[] = {			145.3093,2.3186,7.1824,16.4359,37.2166,37.7803,54.502,-3.5809				};
	double muArray_1[] = {			119.0347,1.0004,10.5113,16.4967,33.4223,33.0245,60.7816,-1.7424				};

	double sigmaArray_0[] = {		71.8871,1.1888,-6.3919,2.1759,-12.8791,-9.7401,-4.9752,-9.5203,1.1888,24.3141,-0.43667,2.3628,7.1202,-2.0591,5.276,-17.3452,-6.3919,-0.43667,1.749,-0.48403,0.34381,1.4717,-0.36001,-0.61024,2.1759,2.3628,-0.48403,1.3684,2.4958,0.27596,1.6541,-1.0475,-12.8791,7.1202,0.34381,2.4958,13.709,5.0835,4.6905,-4.9262,-9.7401,-2.0591,1.4717,0.27596,5.0835,4.4143,-0.20451,-1.2702,-4.9752,5.276,-0.36001,1.6541,4.6905,-0.20451,7.5505,1.6798,-9.5203,-17.3452,-0.61024,-1.0475,-4.9262,-1.2702,1.6798,34.082				};
	double sigmaArray_1[] = {		332.521,-19.9406,-52.7226,53.0436,103.4661,100.5445,66.3989,49.9555,-19.9406,16.4703,3.0369,0.25231,-1.6621,-6.3955,-11.2062,-2.2165,-52.7226,3.0369,10.1815,-9.6832,-18.6553,-18.4258,-11.1193,-8.6307,53.0436,0.25231,-9.6832,17.2169,27.3674,25.0647,11.6776,10.4163,103.4661,-1.6621,-18.6553,27.3674,51.9909,44.9633,25.5945,23.0528,100.5445,-6.3955,-18.4258,25.0647,44.9633,45.9465,24.8972,20.5616,66.3989,-11.2062,-11.1193,11.6776,25.5945,24.8972,24.1067,15.775,49.9555,-2.2165,-8.6307,10.4163,23.0528,20.5616,15.775,20.5605				};


	ICubUtil::putDataIntoVector(muArray_0,numDimensions,mu[0]);
	ICubUtil::putDataIntoVector(muArray_1,numDimensions,mu[1]);


	ICubUtil::putDataIntoMatrix(sigmaArray_0,numDimensions,numDimensions,sigma[0]);
	ICubUtil::putDataIntoMatrix(sigmaArray_1,numDimensions,numDimensions,sigma[1]);

	componentsPrior[0] = 0.4707;
	componentsPrior[1] = 0.5293;


	dbgTag = "GMMData: ";

}


/*
GMMData::GMMData() {
	using iCub::plantIdentification::ICubUtil;

	int numComponents = 2;
	int dimQ = 2;
	int dimR = 6;

	muQ.resize(numComponents);
	muR.resize(numComponents);
	sigmaQQ.resize(numComponents);
	sigmaQQInv.resize(numComponents);
	sigmaQQDet.resize(numComponents);
	sigmaRQ.resize(numComponents);
	sigmaRR.resize(numComponents);
	componentsPrior.resize(numComponents);
	

	double muQArray_0[] = {			145.3093,2.3186				};
	double muQArray_1[] = {			119.0347,1.0004				};

	double muRArray_0[] = {			7.1824,16.4359,37.2166,37.7803,54.502,-3.5809	};
	double muRArray_1[] = {			10.5113,16.4967,33.4223,33.0245,60.7816,-1.7424	};

	double sigmaQQArray_0[] = {		71.8871,1.1888,1.1888,24.3141				};
	double sigmaQQArray_1[] = {		332.521,-19.9406,-19.9406,16.4703				};

    double sigmaRQArray_0[] = {		-6.3919,-0.43667,2.1759,2.3628,-12.8791,7.1202,-9.7401,-2.0591,-4.9752,5.276,-9.5203,-17.3452				};
    double sigmaRQArray_1[] = {		-52.7226,3.0369,53.0436,0.25231,103.4661,-1.6621,100.5445,-6.3955,66.3989,-11.2062,49.9555,-2.2165				};

    double sigmaRRArray_0[] = {		1.749,-0.48403,0.34381,1.4717,-0.36001,-0.61024,-0.48403,1.3684,2.4958,0.27596,1.6541,-1.0475,0.34381,2.4958,13.709,5.0835,4.6905,-4.9262,1.4717,0.27596,5.0835,4.4143,-0.20451,-1.2702,-0.36001,1.6541,4.6905,-0.20451,7.5505,1.6798,-0.61024,-1.0475,-4.9262,-1.2702,1.6798,34.082	};
    double sigmaRRArray_1[] = {		10.1815,-9.6832,-18.6553,-18.4258,-11.1193,-8.6307,-9.6832,17.2169,27.3674,25.0647,11.6776,10.4163,-18.6553,27.3674,51.9909,44.9633,25.5945,23.0528,-18.4258,25.0647,44.9633,45.9465,24.8972,20.5616,-11.1193,11.6776,25.5945,24.8972,24.1067,15.775,-8.6307,10.4163,23.0528,20.5616,15.775,20.5605	};


	ICubUtil::putDataIntoVector(muQArray_0,dimQ,muQ[0]);
	ICubUtil::putDataIntoVector(muQArray_1,dimQ,muQ[1]);

	ICubUtil::putDataIntoVector(muRArray_0,dimR,muR[0]);
	ICubUtil::putDataIntoVector(muRArray_1,dimR,muR[1]);

	ICubUtil::putDataIntoMatrix(sigmaQQArray_0,dimQ,dimQ,sigmaQQ[0]);
	ICubUtil::putDataIntoMatrix(sigmaQQArray_1,dimQ,dimQ,sigmaQQ[1]);

	ICubUtil::putDataIntoMatrix(sigmaRQArray_0,dimR,dimQ,sigmaRQ[0]);
	ICubUtil::putDataIntoMatrix(sigmaRQArray_1,dimR,dimQ,sigmaRQ[1]);

	ICubUtil::putDataIntoMatrix(sigmaRRArray_0,dimR,dimR,sigmaRR[0]);
	ICubUtil::putDataIntoMatrix(sigmaRRArray_1,dimR,dimR,sigmaRR[1]);

	componentsPrior[0] = 0.4707;
	componentsPrior[1] = 0.5293;


	//USING REGRESSION FROM DATA COLLECTED ON THE 3rd of May 2016

	//int numComponents = 2;
	//int dimQ = 2;
	//int dimR = 6;

	//muQ.resize(numComponents);
	//muR.resize(numComponents);
	//sigmaQQ.resize(numComponents);
	//sigmaQQInv.resize(numComponents);
	//sigmaQQDet.resize(numComponents);
	//sigmaRQ.resize(numComponents);
	//sigmaRR.resize(numComponents);
	//componentsPrior.resize(numComponents);
	//

	//double muQArray_0[] = {			125.0963,5.868				};
	//double muQArray_1[] = {			100.2376,-11.2185				};

	//double muRArray_0[] = {			-3.9236,15.4292,15.6348,15.0474,72.0427,2.2902	};
	//double muRArray_1[] = {			5.252,8.5181,8.728,9.2738,72.0909,-1.3128	};

	//double sigmaQQArray_0[] = {		90.2557,12.2637,12.2637,36.6944				};
	//double sigmaQQArray_1[] = {		126.2886,38.6396,38.6396,51.7504				};

	//double sigmaRQArray_0[] = {		19.5466,-8.7572,-5.0825,-6.5077,0.78652,-2.9912,-8.7572,9.2555,6.9184,7.0486,0.13741,0.61024,-5.0825,6.9184,8.1492,6.0691,1.9902,-0.13248,-6.5077,7.0486,6.0691,8.4419,-0.11325,-0.19726,0.78652,0.13741,1.9902,-0.11325,3.8222,0.2467,-2.9912,0.61024,-0.13248,-0.19726,0.2467,1.389	};
	//double sigmaRQArray_1[] = {		2.8393,-4.1632,-4.8453,-4.8613,0.092229,-1.4968,-4.1632,10.874,8.7407,9.5661,-0.97582,2.5306,-4.8453,8.7407,11.2465,10.4047,-1.3558,2.302,-4.8613,9.5661,10.4047,13.6839,-1.891,3.3565,0.092229,-0.97582,-1.3558,-1.891,4.2499,-0.15697,-1.4968,2.5306,2.302,3.3565,-0.15697,3.3841	};

	//double sigmaRRArray_0[] = {		-23.7291,-24.3996,26.8143,6.296,23.2286,1.6072,25.0319,3.3127,2.352,-1.2734,0.13857,4.8075				};
	//double sigmaRRArray_1[] = {		-16.4377,-8.0591,31.2994,10.172,35.5318,10.5963,35.7067,13.2757,-5.2073,1.1318,9.0848,11.0027				};


	//ICubUtil::putDataIntoVector(muQArray_0,dimQ,muQ[0]);
	//ICubUtil::putDataIntoVector(muQArray_1,dimQ,muQ[1]);

	//ICubUtil::putDataIntoVector(muRArray_0,dimR,muR[0]);
	//ICubUtil::putDataIntoVector(muRArray_1,dimR,muR[1]);

	//ICubUtil::putDataIntoMatrix(sigmaQQArray_0,dimQ,dimQ,sigmaQQ[0]);
	//ICubUtil::putDataIntoMatrix(sigmaQQArray_1,dimQ,dimQ,sigmaQQ[1]);

	//ICubUtil::putDataIntoMatrix(sigmaRQArray_0,dimR,dimQ,sigmaRQ[0]);
	//ICubUtil::putDataIntoMatrix(sigmaRQArray_1,dimR,dimQ,sigmaRQ[1]);

	//ICubUtil::putDataIntoMatrix(sigmaRRArray_0,dimR,dimR,sigmaRR[0]);
	//ICubUtil::putDataIntoMatrix(sigmaRRArray_1,dimR,dimR,sigmaRR[1]);

	//componentsPrior[0] = 0.5044;
	//componentsPrior[1] = 0.4956;





	//USING 1 OUTPUT (HAND POSITION)

	//int numComponents = 2;
	//int dimQ = 2;
	//int dimR = 1;

	//muQ.resize(numComponents);
	//muR.resize(numComponents);
	//sigmaQQ.resize(numComponents);
	//sigmaQQInv.resize(numComponents);
	//sigmaQQDet.resize(numComponents);
	//sigmaRQ.resize(numComponents);
	//sigmaRR.resize(numComponents);
	//componentsPrior.resize(numComponents);
	//

	//double muQArray_0[] = {			124.3851,5.1002				};
	//double muQArray_1[] = {			97.4964,-12.7355				};

	//double muRArray_0[] = {			-3.3059	};
	//double muRArray_1[] = {			5.7962	};

	//double sigmaQQArray_0[] = {		102.1704,12.7968,12.7968,37.8599				};
	//double sigmaQQArray_1[] = {		62.8856,15.2491,15.2491,40.7145				};

	//double sigmaRQArray_0[] = {		-25.6851,-25.2709	};
	//double sigmaRQArray_1[] = {		-5.7768,-3.02	};

	//double sigmaRRArray_0[] = {		20.4438				};
	//double sigmaRRArray_1[] = {		0.8562				};


	//ICubUtil::putDataIntoVector(muQArray_0,dimQ,muQ[0]);
	//ICubUtil::putDataIntoVector(muQArray_1,dimQ,muQ[1]);

	//ICubUtil::putDataIntoVector(muRArray_0,dimR,muR[0]);
	//ICubUtil::putDataIntoVector(muRArray_1,dimR,muR[1]);

	//ICubUtil::putDataIntoMatrix(sigmaQQArray_0,dimQ,dimQ,sigmaQQ[0]);
	//ICubUtil::putDataIntoMatrix(sigmaQQArray_1,dimQ,dimQ,sigmaQQ[1]);

	//ICubUtil::putDataIntoMatrix(sigmaRQArray_0,dimR,dimQ,sigmaRQ[0]);
	//ICubUtil::putDataIntoMatrix(sigmaRQArray_1,dimR,dimQ,sigmaRQ[1]);

	//ICubUtil::putDataIntoMatrix(sigmaRRArray_0,dimR,dimR,sigmaRR[0]);
	//ICubUtil::putDataIntoMatrix(sigmaRRArray_1,dimR,dimR,sigmaRR[1]);

	//componentsPrior[0] = 0.5683;
	//componentsPrior[1] = 0.4317;

	
	for (size_t i = 0; i < sigmaQQ.size(); i++){ 
		sigmaQQInv[i] = yarp::math::luinv(sigmaQQ[i]);
		sigmaQQDet[i] = yarp::math::det(sigmaQQ[i]);
	}

	dbgTag = "GMMData: ";

}
*/

double GMMData::calculateGMProbability(yarp::sig::Vector &queryPoint,int gmComponent){
	using namespace yarp::math;

	const double e = 2.71828183;

	double exp = -0.5 * dot(queryPoint - muQ[gmComponent],sigmaQQInv[gmComponent] * (queryPoint - muQ[gmComponent]));

	return 1/sqrt(sigmaQQDet[gmComponent]* pow(2*3.1415,(double)queryPoint.length())) * pow(e,exp);

}

void GMMData::runGaussianMixtureRegression(yarp::sig::Vector &queryPoint,yarp::sig::Vector &output){
	using namespace yarp::math;

	std::vector<double> h;
	std::vector<double> hNumerator;

	int numGMComponents = muQ.size();

	h.resize(numGMComponents);
	hNumerator.resize(numGMComponents);

	for (size_t i = 0; i < numGMComponents; i++){

		hNumerator[i] = componentsPrior[i] * calculateGMProbability(queryPoint,i);
	}

	double hDenominator = 0;
	for (size_t i = 0; i < numGMComponents; i++){

		hDenominator = hDenominator + hNumerator[i];
	}

	for (size_t i = 0; i < numGMComponents; i++){
	
		h[i] = hNumerator[i]/hDenominator;
	}

	
	output.resize(muR[0].length(),0.0);

	for (size_t i = 0; i < numGMComponents; i++){

		output = output + h[i] * (muR[i] + (sigmaRQ[i] * (sigmaQQInv[i] * (queryPoint - muQ[i]))));
	}

}



void GMMData::buildQRStructures(std::vector<int> &qIndexes,std::vector<int> &rIndexes){

	int dimQ = qIndexes.size();
	int dimR = rIndexes.size();

	for(size_t i = 0; i < numComponents; i++){
		
		ICubUtil::putSelectedElementsIntoVector(mu[i],qIndexes,muQ[i]);
		ICubUtil::putSelectedElementsIntoVector(mu[i],rIndexes,muR[i]);

		ICubUtil::putSelectedElementsIntoMatrix(sigma[i],qIndexes,qIndexes,sigmaQQ[i]);
		ICubUtil::putSelectedElementsIntoMatrix(sigma[i],rIndexes,qIndexes,sigmaRQ[i]);
		ICubUtil::putSelectedElementsIntoMatrix(sigma[i],rIndexes,rIndexes,sigmaRR[i]);

		sigmaQQInv[i] = yarp::math::luinv(sigmaQQ[i]);
		sigmaQQDet[i] = yarp::math::det(sigmaQQ[i]);
	}

//	for(size_t i = 0; i < numComponents; i++){
//       std::cout << "COMPONENT: " << i << "\n\n";
//        std::cout << mu[i].toString() << "\n\n" << muQ[i].toString() << "\n\n" << muR[i].toString() << "\n\n";
//        std::cout << "\n\n";
//        std::cout << sigma[i].toString() << "\n\n" << sigmaQQ[i].toString() << "\n\n" << sigmaRQ[i].toString() << "\n\n" << sigmaRR[i].toString() << "\n\n";
//	}
}
