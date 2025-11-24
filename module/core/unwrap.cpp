#include "fsl/core/unwrap.h"


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

        long long gcd(long long a, long long b) {
            a = std::abs(a);
            b = std::abs(b);
            while (b != 0) {
                long long r = a % b;
                a = b;
                b = r;
            }
            return a;
        }

        long long lcm(long long a, long long b) {
        if (a == 0 || b == 0) {
                return 0;
            }
            long long g = gcd(a, b);
            return std::abs(a / g * b);
        }

        int getLutValue(const std::unordered_map<int, int>& lut, int key) {
            auto it = lut.find(key);
            if (it != lut.end()) {
                return it->second;
            }
            return 0; 
        }
        
    }


    // =======================================
    // Temporal phase unwrap
    // =======================================
    Phase HierarchicalUnwrap(const std::vector<Phase>& wraps,double* ratio,const int ratio_count)
    {
        if (wraps.empty() || !ratio || ratio_count < 1 ||
        ratio_count + 1 != static_cast<int>(wraps.size()))
        {
            std::cerr << "Warning: invalid args.\n";
            return createEmptyPhase();
        }

        Phase unwrap;
        unwrap.rows = wraps[0].rows;
        unwrap.cols = wraps[0].cols;
        unwrap.data = wraps[0].data;
        
        const double scale = 1.0/(2.0*M_PI);

        for (int i = 1; i < wraps.size(); ++i) {
            for (int n = 0; n < unwrap.data.size(); ++n) {
                unwrap.data[n] = wraps[i].data[n] + 2.0*M_PI*round((ratio[i-1]*unwrap.data[n]-wraps[i].data[n])*scale);
            } 
        }

        if (!wraps[wraps.size()-1].B.empty()) {
            unwrap.B = wraps[wraps.size()-1].B;
        }

        return unwrap;
    }

    std::unordered_map<int, int> DualFreNumberTheoreticalLUT(int highFre,int lowFre)
    {

        std::unordered_map<int, int> data; 

        long long lcmFre = lcm(highFre,lowFre);

        int  unambiguousRange  = static_cast<int>(lcmFre);

        std::cout<< "unambiguousRange: "<< unambiguousRange << std::endl;

        for(int i = 0;i<=unambiguousRange;i++)
        {
            double w1 = 2*M_PI*i/highFre-round((2*M_PI*i*1/highFre-M_PI)/(2*M_PI))*2*M_PI;
            double w2 = 2*M_PI*i/lowFre-round((2*M_PI*i*1/lowFre-M_PI)/(2*M_PI))*2*M_PI;

            int key = static_cast<int>(round((highFre * w1 - lowFre * w2) / (2*M_PI)));
            int value = static_cast<int>(round((2*M_PI*i*1/highFre-M_PI)/(2*M_PI)));

            data[key] = value;
            std::cout<<" key: "<<key<<" value: "<<value<<std::endl;
        }

        return data;

    }

    Phase DualFreNumberTheoreticalUnwrap(const std::vector<Phase>& wraps, const std::unordered_map<int, int>& lut,const int highFre,const int lowFre)
    {
        if (wraps.empty() || wraps.size() != 2)
        {
            std::cerr << "Warning: invalid args.\n";
            return createEmptyPhase();
        }

        Phase unwrap;
        unwrap.rows = wraps[0].rows;
        unwrap.cols = wraps[0].cols;
        unwrap.data = wraps[0].data;


        for(int i = 0;i < unwrap.data.size();i++)
        {
            int key = static_cast<int>(round((highFre * wraps[1].data[i] - lowFre * wraps[0].data[i]) / (2*M_PI)));
            int value = getLutValue(lut,key);
            unwrap.data[i] = value;
        }

        if (!wraps[wraps.size()-1].B.empty()) {
            unwrap.B = wraps[wraps.size()-1].B;
        }

        return unwrap;
        
    }

    std::pair<Phase,Phase> DualFreNumberTheoreticalUnwrapAdValue(const std::vector<Phase>& wraps, const std::unordered_map<int, int>& lut,const int highFre,const int lowFre)
    {
        if (wraps.empty() || wraps.size() != 2)
        {
            std::cerr << "Warning: invalid args.\n";
            return std::make_pair(createEmptyPhase(),createEmptyPhase());
        }

        Phase unwrap;
        unwrap.rows = wraps[0].rows;
        unwrap.cols = wraps[0].cols;
        unwrap.data = wraps[0].data;

        Phase Value;
        Value.rows = wraps[0].rows;
        Value.cols = wraps[0].cols;
        Value.data = wraps[0].data;

        for(int i = 0;i < unwrap.data.size();i++)
        {
            int key = static_cast<int>(round((highFre * wraps[1].data[i] - lowFre * wraps[0].data[i]) / (2*M_PI)));
            int value = getLutValue(lut,key);
            Value.data[i] = key;
            unwrap.data[i] = wraps[1].data[i] + 2*M_PI*value;
        }

        if (!wraps[wraps.size()-1].B.empty()) {
            unwrap.B = wraps[wraps.size()-1].B;
        }

        return std::make_pair(unwrap,Value);

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