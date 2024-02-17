#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"
#include <iostream>

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() : Sampler(),
                                     randDistribution(0, 1),
                                     normalDistribution(0, 1),
                                     randEngine(::std::random_device()())
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
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

        ::rl::math::Vector
        YourSampler::generateGaussian()
        {
            ::rl::math::Vector sampleq(this->model->getDof());
            bool valid = false;

            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());

            while (!valid)
            {
                // Generate Sample
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    sampleq(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                }
                // Check if sample is valid
                ::rl::math::Vector sampleqgauss(this->model->getDof());
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    sampleqgauss(i) = sampleq(i) + this->gauss() * this->sigma(i);
                }
                this->model->clip(sampleqgauss);
                this->model->setPosition(sampleqgauss);
                this->model->updateFrames();
                valid = this->model->isColliding();
            }

            return sampleq;
        }

        ::rl::math::Vector
        YourSampler::generateBridge()
        {
            ::rl::math::Vector sampleq(this->model->getDof());
            bool valid = false;

            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());

            while (!valid)
            {
                // Generate Sample
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    sampleq(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                }

                this->model->clip(sampleq);
                this->model->setPosition(sampleq);
                this->model->updateFrames();
                if (this->model->isColliding())
                {
                    continue;
                }

                // Check if sample is valid
                ::rl::math::Vector sampleqgaussl(this->model->getDof());
                ::rl::math::Vector sampleqgaussr(this->model->getDof());
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    sampleqgaussr(i) = sampleq(i) + this->gauss() * this->sigma(i);
                    sampleqgaussl(i) = sampleq(i) - this->gauss() * this->sigma(i);
                }

                this->model->clip(sampleqgaussl);
                this->model->setPosition(sampleqgaussl);
                this->model->updateFrames();
                bool validl = this->model->isColliding();
                if (!validl)
                {
                    continue;
                }

                this->model->clip(sampleqgaussr);
                this->model->setPosition(sampleqgaussr);
                this->model->updateFrames();
                bool validr = this->model->isColliding();
                if (validr)
                {
                    valid = true;
                }
            }

            return sampleq;
        }

        ::rl::math::Vector
        YourSampler::generateGaussianAlongCPath(const Eigen::MatrixXd &Q, const double lengthStartGoal)
        {
            // Q.col(0) is the direction vector from "start" to "goal"
            // we multiply this by a random length in [0, length(start->goal)]
            ::rl::math::Vector sampleq = Q.col(0) * (this->rand() * lengthStartGoal); // Initial point along A-B

            for (int i = 1; i < Q.cols(); ++i)
            {
                double gaussianSample = this->gauss(); // Gaussian sample for direction

                // TODO add variance

                sampleq += Q.col(i) * gaussianSample; // Adjust position in each basis direction
            }

            this->model->clip(sampleq); // Ensure within model limits
            this->model->setPosition(sampleq);
            this->model->updateFrames();

            return sampleq;
        }

        ::rl::math::Vector
        YourSampler::generateGaussianAlongCPath_Improved(const Eigen::MatrixXd &Q, const double lengthStartGoal, ::std::vector<double> furthestDistanceFromOrigin)
        {
            // Q.col(0) is the direction vector from "start" to "goal"

            // THIS IS WHAT WE IMPROVED


            ::rl::math::Vector sampleq;

            double sigma_for_line = 1;
            double sigma_for_directions = 1;

            // 50/50 Chance (equally likely)
            if (this->rand() >= 0.5)
            {
                // FOR TREE 0

                // sample from gauss curve that is shifted to have mean
                //   at furthestDistanceFromOrigin for first tree
                double gauss_sample_along_cpath = this->gauss()*sigma_for_line + furthestDistanceFromOrigin[0];
                
                if(gauss_sample_along_cpath<0)
                {
                    gauss_sample_along_cpath=0;
                }

                // multiply unit vector of start->goal direction with this length
                sampleq = Q.col(0) * gauss_sample_along_cpath;
            }
            else
            {
                // FOR TREE 1
                // sample from gauss curve that is shifted to have mean
                //   at furthestDistanceFromOrigin for second tree
                double gauss_sample_along_cpath = this->gauss()*sigma_for_line + furthestDistanceFromOrigin[1];

                if(gauss_sample_along_cpath<0)
                {
                    gauss_sample_along_cpath=0;
                }

                // subtract this distance from goal vector
                sampleq = Q.col(0) * (lengthStartGoal - gauss_sample_along_cpath);
            }

            for (int i = 1; i < Q.cols(); ++i)
            {
                double gaussianSample = this->gauss() * sigma_for_directions; // Gaussian sample for direction
                sampleq += Q.col(i) * gaussianSample; // Adjust position in each basis direction
            }

            this->model->clip(sampleq); // Ensure within model limits

            return sampleq;
        }

        ::std::uniform_real_distribution<::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        ::std::normal_distribution<::rl::math::Real>::result_type
        YourSampler::gauss()
        {
            return this->normalDistribution(this->randEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type &value)
        {
            this->randEngine.seed(value);
        }

        void
        YourSampler::setSigma(const ::rl::math::Real delta)
        {
            this->sigma = ::rl::math::Vector(this->model->getDof());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                this->sigma(i) = delta;
            }
        }
    }
}
