#pragma once
#include <opencv2/core.hpp>
#include <map>
#include <memory>

namespace ols {
    class Gamma {
    public:
        Gamma(float value);
        void apply(cv::Mat &m) const;
    private:
        template<int G>
        void apply_v(float*ptr,int total) const;
        static constexpr int N=6;
        int roots;
        float p[N];
        float gamma;
    };

    class CachedGamma {
    public:
        void apply(cv::Mat &m,float val)
        {
            auto p=g_.find(val);
            if(p!=g_.end()) {
                p->second->apply(m);
            }
            else {
                std::shared_ptr<Gamma> gp(new Gamma(val));
                gp->apply(m);
                g_[val]=gp;
            }
        }
    private:
        std::map<float,std::shared_ptr<Gamma> > g_;
    };
}

