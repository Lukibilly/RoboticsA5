#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            Sampler(),
            randDistribution(0, 1),
            randEngine(::std::random_device()())
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            // Our template code performs uniform sampling.
            // You are welcome to change any or all parts of the sampler.
            // BUT PLEASE MAKE SURE YOU CONFORM TO JOINT LIMITS,
            // AS SPECIFIED BY THE ROBOT MODEL!

            ::rl::math::Vector sampleq(this->model->getDof());

            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                sampleq(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
            }

            // It is a good practice to generate samples in the
            // the allowed configuration space as done above.
            // Alternatively, to make sure generated joint 
            // configuration values are clipped to the robot model's 
            // joint limits, you may use the clip() function like this: 
            // this->model->clip(sampleq);

            return sampleq;
        }

        ::std::uniform_real_distribution< ::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->randEngine.seed(value);
        }
    }
}

