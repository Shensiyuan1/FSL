#include "fsl/unwrap.h"


namespace Fringe{
    namespace{
        Phase createEmptyPhase() 
        {
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;
        }

        struct CompareByValue {
            bool operator()(const std::pair<int, int>& a, const std::pair<int, int>& b) const {
                        return a.second < b.second; 
                    }
                };
    }









    // =======================================
    // Space phase unwrap
    // =======================================
    std::pair<Phase,std::unordered_map<int,int>> segphase(const Phase& phase,const std::vector<double>& mask,std::string mode,int thresh,double seg_thresh)
    {

        if (phase.data.empty()) {
            std::cerr << "Warning: empty phase.\n";
            return std::pair<Phase, std::unordered_map<int, int>>(createEmptyPhase(),{});
        }

        const int width = phase.cols;
	    const int height = phase.rows;
        const size_t total = static_cast<size_t>(width) * height;

        std::vector<double> local_mask;
        local_mask.assign(total, 0.0); 

        if (mode == "modulation" || mode == "mask") {
            if (mask.size() != total) {
                std::cerr << "Warning: mask size (" << mask.size()
                        << ") does not match phase size (" << total << ").\n";
                return {createEmptyPhase(), {}};
            }

            if (mode == "modulation") {
                for (size_t i = 0; i < total; ++i) {
                    if (mask[i] > thresh) {         
                        local_mask[i] = 1;
                    }
                }
            } else { 
                for (size_t i = 0; i < total; ++i) {
                    local_mask[i] = mask[i];
                }
            }
        }
        else if (mode == "phase") {
            for (size_t i = 0; i < total; ++i) {
                if (phase.data[i] != 0.0) {
                    local_mask[i] = 1;
                }
            }
        }
        else if (mode == "all") {
            std::fill(local_mask.begin(), local_mask.end(), 1); 
        }
        else {
            std::cerr << "Error: unsupported mode '" << mode << "'. "
                    << "Supported: all, phase, modulation, mask.\n";
            return {createEmptyPhase(), {}};
        }


        std::unordered_map<int, int> count;

        Phase segment;
        segment.rows = height;
        segment.cols = width;
        segment.data.resize(width * height, 0.0);

        int seg = 0; //init segment number

        // 4 neighborhood 
        const int X[4] = {-1,0,0,1}; 
        const int Y[4] = {0,-1,1,0};

        for (size_t i = 0; i < total; i++)
        {
            if(local_mask[i] == 0)
            {
                segment.data[i] = 0.0;
            }
            if (local_mask[i] != 0 && segment.data[i] == 0) 
            {
                seg += 1;
                segment.data[i] = seg;
                count[seg] = 1;

                std::queue<int> grow;
                grow.push(i);

                while (!grow.empty()) {
                    int idx = grow.front();
                    grow.pop();
                    double current_phase = phase.data[idx];

                    for (int j = 0; j < 4; ++j) {
                        int x = idx % width + X[j];
                        int y = idx / width + Y[j];
                        if (x < 0 || x >= width || y < 0 || y >= height) continue;

                        int nidx = y * width + x;
                        if (local_mask[nidx] == 0 || segment.data[nidx] != 0) continue;

                        if (abs(current_phase - phase.data[nidx]) < seg_thresh) {
                            segment.data[nidx] = seg;
                            count[seg]++;
                            grow.push(nidx);
                        }
                    }
                }

            }

        }

        return std::pair<Phase, std::unordered_map<int, int>>(segment,count);

    }

    void optPhaseBYvalue(Phase &phase,Phase& value,Phase& segment,const std::unordered_map<int,int>& lut)
    {
        if (phase.data.empty()) {
            std::cerr << "Warning: empty phase.\n";
            return ;
        }

        const int width = phase.cols;
        const int height = phase.rows;
        const size_t total = static_cast<size_t>(width) * height;

        const int X[4] = {-1,0,0,1}; 
        const int Y[4] = {0,-1,1,0};

        std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>,CompareByValue> q;

        for (std::unordered_map<int, int>::const_iterator it = lut.begin(); it != lut.end(); ++it) {
            if (it->first != 0) {
                q.push(std::make_pair(it->first, it->second));
            }
        }

        while (!q.empty()) {
            std::pair<int, int> p = q.top();
            int seg = p.first;
            int number = p.second;

            //std::cout<<"seg: "<<seg<<" number: "<<number<<std::endl;

            q.pop();

            for (size_t i = 0; i < total; i++) {

                if (static_cast<int>(segment.data[i]) != seg) continue;

                for (int j = 0; j < 4; ++j) {

                    int x = static_cast<int>(i % width) + X[j];
                    int y = static_cast<int>(i / width) + Y[j];
                    if (x < 0 || x >= width || y < 0 || y >= height) continue;

                    int nidx = y * width + x;
                    int neighbor_seg = static_cast<int>(segment.data[nidx]);

                    if (neighbor_seg == 0 || neighbor_seg == seg) continue;

                    if (abs(value.data[i] - value.data[nidx]) > 1.0 ) continue;

                    auto it = lut.find(neighbor_seg);
                    if (it == lut.end()) continue;

                    double ratio = it->second / static_cast<double>(number);
                    if ( ratio <= 0.1) {

                        value.data[nidx] = value.data[i];
                        segment.data[nidx] = seg;
                        phase.data[nidx] = phase.data[nidx] + 2*M_PI*round((phase.data[i] - phase.data[nidx])/(2*M_PI));
                    }
                }
            }
        }

            

    }
  

}