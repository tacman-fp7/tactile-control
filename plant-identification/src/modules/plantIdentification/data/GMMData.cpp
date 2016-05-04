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
