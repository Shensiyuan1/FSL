#include <fsl/unwrap.h>


namespace Fringe{
    namespace{
        Phase createEmptyPhase() 
        {
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;
        }
    }

    std::pair<Phase,std::unordered_map<int,int>> segphase(const Phase& phase,const std::vector<double>& mask,std::string mode,int thresh)
    {

        if (phase.data.empty()) {
            std::cerr << "Warning: empty phase.\n";
            return std::pair<Phase, std::unordered_map<int, int>>(createEmptyPhase(),{});
        }

        const int width = phase.cols;
	    const int height = phase.rows;
        const size_t total = static_cast<size_t>(width) * height;

        std::vector<double> local_mask;
        local_mask.resize(width * height, 0.0); 

        if (mode == "modulation") {
            if (mask.size() != phase.data.size()) {
                std::cerr << "Warning: mask size is not equal to phase size.\n";
                return {createEmptyPhase(), {}};
            }
            for (size_t i = 0; i < total; ++i) {
                if (mask[i] > thresh) {
                    local_mask[i] = 1;
                }
            }
        }


        std::unordered_map<int, int> count;
        count[1] = -1;

        Phase segment;
        segment.rows = height;
        segment.cols = width;
        segment.data.resize(width * height, 0.0);

        int seg = 1; //init segment number

        // 4 neighborhood 
        const int X[4] = {-1,0,0,1}; 
        const int Y[4] = {0,-1,1,0};

        for (size_t i = 0; i < total; i++)
        {

        }






        return std::pair<Phase, std::unordered_map<int, int>>(segment,count);

    }

}