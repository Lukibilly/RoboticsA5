#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/Sampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Uniform random sampling strategy.
         */
        class YourSampler : public Sampler
        {
        public:
            YourSampler();

            virtual ~YourSampler();

            ::rl::math::Vector generate();
            ::rl::math::Vector generateGaussian();
            ::rl::math::Vector generateBridge();

            virtual void seed(const ::std::mt19937::result_type& value);
            void setSigma(const ::rl::math::Real delta);
            
            ::rl::math::Vector sigma;

        protected:
            ::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();
            ::std::normal_distribution< ::rl::math::Real>::result_type gauss();

            ::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
            ::std::normal_distribution< ::rl::math::Real> normalDistribution;

            ::std::mt19937 randEngine;

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_
