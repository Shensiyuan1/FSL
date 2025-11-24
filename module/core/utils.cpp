#include "fsl/utils.h"

namespace Fringe
{
    namespace 
    {
        Phase createEmptyPhase() 
        {
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;
        }

        PointCloud createEmptyPointCloud() 
        {
            PointCloud empty;
            return empty;
        }

    }
    

}


