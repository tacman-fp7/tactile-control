#include "iCub/plantIdentification/data/GMMData.h"

#include "iCub/plantIdentification/util/ICubUtil.h"

#include <yarp/math/Math.h>
#include <math.h>

using iCub::plantIdentification::GMMData;

using yarp::os::Bottle;

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
	

	double muQArray_0[] = {			125.0963,5.868				};
	double muQArray_1[] = {			100.2376,-11.2185				};

	double muRArray_0[] = {			-3.9236,15.4292,15.6348,15.0474,72.0427,2.2902	};
	double muRArray_1[] = {			5.252,8.5181,8.728,9.2738,72.0909,-1.3128	};

	double sigmaQQArray_0[] = {		90.2557,12.2637,12.2637,36.6944				};
	double sigmaQQArray_1[] = {		126.2886,38.6396,38.6396,51.7504				};

	double sigmaRQArray_0[] = {		19.5466,-8.7572,-5.0825,-6.5077,0.78652,-2.9912,-8.7572,9.2555,6.9184,7.0486,0.13741,0.61024,-5.0825,6.9184,8.1492,6.0691,1.9902,-0.13248,-6.5077,7.0486,6.0691,8.4419,-0.11325,-0.19726,0.78652,0.13741,1.9902,-0.11325,3.8222,0.2467,-2.9912,0.61024,-0.13248,-0.19726,0.2467,1.389	};
	double sigmaRQArray_1[] = {		2.8393,-4.1632,-4.8453,-4.8613,0.092229,-1.4968,-4.1632,10.874,8.7407,9.5661,-0.97582,2.5306,-4.8453,8.7407,11.2465,10.4047,-1.3558,2.302,-4.8613,9.5661,10.4047,13.6839,-1.891,3.3565,0.092229,-0.97582,-1.3558,-1.891,4.2499,-0.15697,-1.4968,2.5306,2.302,3.3565,-0.15697,3.3841	};

	double sigmaRRArray_0[] = {		-23.7291,-24.3996,26.8143,6.296,23.2286,1.6072,25.0319,3.3127,2.352,-1.2734,0.13857,4.8075				};
	double sigmaRRArray_1[] = {		-16.4377,-8.0591,31.2994,10.172,35.5318,10.5963,35.7067,13.2757,-5.2073,1.1318,9.0848,11.0027				};


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

	componentsPrior[0] = 0.5044;
	componentsPrior[1] = 0.4956;


	/* 

	USING 1 OUTPUT (HAND POSITION)

	int numComponents = 2;
	int dimQ = 2;
	int dimR = 1;

	muQ.resize(numComponents);
	muR.resize(numComponents);
	sigmaQQ.resize(numComponents);
	sigmaQQInv.resize(numComponents);
	sigmaQQDet.resize(numComponents);
	sigmaRQ.resize(numComponents);
	sigmaRR.resize(numComponents);
	componentsPrior.resize(numComponents);
	

	double muQArray_0[] = {			124.3851,5.1002				};
	double muQArray_1[] = {			97.4964,-12.7355				};

	double muRArray_0[] = {			-3.3059	};
	double muRArray_1[] = {			5.7962	};

	double sigmaQQArray_0[] = {		102.1704,12.7968,12.7968,37.8599				};
	double sigmaQQArray_1[] = {		62.8856,15.2491,15.2491,40.7145				};

	double sigmaRQArray_0[] = {		-25.6851,-25.2709	};
	double sigmaRQArray_1[] = {		-5.7768,-3.02	};

	double sigmaRRArray_0[] = {		20.4438				};
	double sigmaRRArray_1[] = {		0.8562				};


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

	componentsPrior[0] = 0.5683;
	componentsPrior[1] = 0.4317;

	*/
	for (size_t i = 0; i < sigmaQQ.size(); i++){ 
		sigmaQQInv[i] = yarp::math::luinv(sigmaQQ[i]);
		sigmaQQDet[i] = yarp::math::det(sigmaQQ[i]);
	}

	dbgTag = "GMMData: ";

}

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
