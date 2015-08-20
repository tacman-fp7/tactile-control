#ifndef __ICUB_PLANTIDENTIFICATION_ICUBUTIL_H__
#define __ICUB_PLANTIDENTIFICATION_ICUBUTIL_H__

#include <iCub/plantIdentification/PlantIdentificationEnums.h>

#include <vector>

namespace iCub {
    namespace plantIdentification {

        class ICubUtil {
    
		public:

				static double getForce(std::vector<double>& tactileData,iCub::plantIdentification::ForceCalculationMode forceCalculationMode);

		private:

				static double getForceBySimpleSum(std::vector<double>& tactileData);

				static double getForceByWeightedSum(std::vector<double>& tactileData);
				static void getUnitVector(int index,std::vector<double>& unitVector);

				static double getForceByLearntMapping(std::vector<double>& tactileData);

};
    } //namespace plantIdentification
} //namespace iCub

#endif

