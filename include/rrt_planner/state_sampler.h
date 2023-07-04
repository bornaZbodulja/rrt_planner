/**
 * @file state_sampler.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State sampler implementation
 * @version 0.1
 * @date 2023-07-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__STATE_SAMPLER_H_
#define RRT_PLANNER__STATE_SAMPLER_H_

#include <experimental/random>

#include "rrt_planner/node_2d.h"
#include "rrt_planner/node_hybrid.h"

namespace rrt_planner {

template <typename NodeT>
class StateSampler {
 public:
  /**
   * @brief Holder for index generation parameters
   */
  struct SamplerParams {
    SamplerParams() {}
    SamplerParams(const double& target_bias_in,
                  const unsigned int& state_space_size_in,
                  const double& rgd_increment_step_in,
                  const unsigned char& rgd_stop_cost_in,
                  const int& rgd_iterations_in)
        : target_bias(target_bias_in),
          state_space_size(state_space_size_in),
          rgd_increment_step(rgd_increment_step_in),
          rgd_stop_cost(rgd_stop_cost_in),
          rgd_iterations(rgd_iterations_in) {}

    double target_bias{0.0};
    unsigned int state_space_size{0};
    double rgd_increment_step{0.0};
    unsigned char rgd_stop_cost{0};
    int rgd_iterations{0};
  };
  /**
   * @brief Constructor for state sampler
   * @param collision_checker Collision checker pointer
   */
  explicit StateSampler(const CollisionCheckerPtr& collision_checker);

  /**
   * @brief Destructor for state sampler
   */
  ~StateSampler(){};

  /**
   * @brief Updates collision checker for sampler
   * @param collision_checker
   */
  inline void UpdateCollisionChecker(
      const CollisionCheckerPtr& collision_checker) {
    collision_checker_ = collision_checker;
  }

  /**
   * @brief Updates sampler parameters
   * @param target_bias_in Bias towards target of search tree when selecting new
   * node for expansion
   * @param state_space_size_in State space size
   * @param rgd_increment_step_in Increment step for random gradient descent
   * @param rgd_stop_cost_in Stop cost for random gradient descent
   * @param rgd_iterations_in Number of iterations for random gradient descent
   */
  inline void UpdateSamplerParams(const double& target_bias_in,
                                  const unsigned int& state_space_size_in,
                                  const double& rgd_increment_step_in,
                                  const unsigned char& rgd_stop_cost_in,
                                  const int& rgd_iterations_in) {
    params_.target_bias = target_bias_in;
    params_.state_space_size = state_space_size_in;
    params_.rgd_increment_step = rgd_increment_step_in;
    params_.rgd_stop_cost = rgd_stop_cost_in;
    params_.rgd_iterations = rgd_iterations_in;
  }

  /**
   * @brief Gets new index for tree expansion
   * @param target_index Tree target index
   * @return unsigned int
   */
  unsigned int operator()(const unsigned int& target_index);

 private:
  /**
   * @brief Random gradient descent towards target
   * @param random_index Randomly generated index
   * @param target_index Index of target node
   * @return unsigned int
   */
  unsigned int RGD(const unsigned int& random_index,
                   const unsigned int& target_index);

  /**
   * @brief Generates random index in state space
   * @param state_space_size State space size
   * @return unsigned int Random index
   */
  unsigned int GenerateRandomIndex(const unsigned int& state_space_size) const;

  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;
  // Sampler parameters
  SamplerParams params_;
  // Utils for random number generation
  std::minstd_rand gen_;
  std::uniform_real_distribution<double> dist_;
};

}  // namespace rrt_planner

#endif
