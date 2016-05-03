#include "iCub/plantIdentification/data/GMMData.h"

#include "iCub/plantIdentification/util/ICubUtil.h"

#include <yarp/math/Math.h>

using iCub::plantIdentification::GMMData;

using yarp::os::Bottle;

GMMData::GMMData() {
	using iCub::plantIdentification::ICubUtil;

	int numComponents = 1;
	int dimQ = 2;
	int dimR = 7;

	muQ.resize(numComponents);
	muR.resize(numComponents);
	sigmaQQ.resize(numComponents);
	sigmaQQInv.resize(numComponents);
	sigmaQQDet.resize(numComponents);
	sigmaRQ.resize(numComponents);
	sigmaRR.resize(numComponents);
	componentsPrior.resize(numComponents);
	

	double muQArray_0[] = {			6.3636,4.7273				};

	double muRArray_0[] = {			4.1818,3.4545,4.6364,4.6364,5.7273,5.3636,6	};

	double sigmaQQArray_0[] = {		31.1405,-1.0826,-1.0826,2.0165				};

	double sigmaRQArray_0[] = {		-5.7025,1.595,-2.5289,0.30579,-1.7769,-1.0083,-5.3223,-0.82645,5.6446,-0.34711,2.7769,0.28099,-6.3636,0.090909	};

	double sigmaRRArray_0[] = {		4.876,2.7355,-1.5702,-0.024793,-0.8595,0.38843,3.0909,2.7355,3.7025,0.80165,0.98347,-0.69421,-0.52893,1.9091,-1.5702,0.80165,6.0496,3.1405,-0.19008,0.041322,-1.0909,-0.024793,0.98347,3.1405,3.8678,-1.0992,0.22314,1.5455,-0.8595,-0.69421,-0.19008,-1.0992,3.6529,1.4628,-1.4545,0.38843,-0.52893,0.041322,0.22314,1.4628,4.7769,2.2727,3.0909,1.9091,-1.0909,1.5455,-1.4545,2.2727,6.7273				};


	ICubUtil::putDataIntoVector(muQArray_0,dimQ,muQ[0]);

	ICubUtil::putDataIntoVector(muRArray_0,dimR,muR[0]);

	ICubUtil::putDataIntoMatrix(sigmaQQArray_0,dimQ,dimQ,sigmaQQ[0]);

	ICubUtil::putDataIntoMatrix(sigmaRQArray_0,dimR,dimQ,sigmaRQ[0]);

	ICubUtil::putDataIntoMatrix(sigmaRRArray_0,dimR,dimR,sigmaRR[0]);

	componentsPrior[0] = 1.0;

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
		yarp::sig::Vector a;
		yarp::sig::Vector b = sigmaQQInv[i] * (queryPoint - muQ[i]);
		yarp::sig::Vector c = sigmaRQ[i] * b;
		yarp::sig::Vector d = muR[i] + c;
		yarp::sig::Vector e = h[i] * d;
		yarp::sig::Vector f = output + e;

		output = output + h[i] * (muR[i] + (sigmaRQ[i] * (sigmaQQInv[i] * (queryPoint - muQ[i]))));
	}


}
