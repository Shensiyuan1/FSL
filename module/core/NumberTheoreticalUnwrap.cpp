#include "fsl/NumberTheoreticalUnwrap.h"


namespace Fringe
{
    namespace
    {
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

        Phase createEmptyPhase() 
        {
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;
        }
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
            unwrap.data[i] = value;
        }

        if (!wraps[wraps.size()-1].B.empty()) {
            unwrap.B = wraps[wraps.size()-1].B;
        }

        return std::make_pair(unwrap,Value);

    }
    


    



}
