/**
 * @file index_generator.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Index generator implementation
 * @version 0.1
 * @date 2023-07-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__INDEX_GENERATOR_H_
#define RRT_PLANNER__INDEX_GENERATOR_H_

#include <experimental/random>

#include "nav_utils/nav_utils.h"
#include "rrt_planner/node_2d.h"
#include "rrt_planner/node_hybrid.h"

namespace rrt_planner {

template <typename NodeT>
class IndexGenerator {
 public:
  /**
   * @brief Holder for index generation parameters
   */
  struct GeneratorParams {
    GeneratorParams() {}
    GeneratorParams(const double& target_bias_in,
                    const unsigned int& state_space_size_in)
        : target_bias(target_bias_in), state_space_size(state_space_size_in) {}

    double target_bias{0.0};
    unsigned int state_space_size{0};
  };
  /**
   * @brief Constructor for index generator
   * @param collision_checker Collision checker pointer
   */
  explicit IndexGenerator(const CollisionCheckerPtr& collision_checker);

  /**
   * @brief Destructor for index generator
   */
  ~IndexGenerator(){};

  /**
   * @brief Updates collision checker for generator
   * @param collision_checker
   */
  inline void UpdateCollisionChecker(
      const CollisionCheckerPtr& collision_checker) {
    collision_checker_ = collision_checker;
  }

  /**
   * @brief Updates generator parameters
   * @param target_bias_in
   * @param state_space_size_in
   */
  inline void UpdateGeneratorParams(const double& target_bias_in,
                                    const unsigned int& state_space_size_in) {
    params_.target_bias = target_bias_in;
    params_.state_space_size = state_space_size_in;
  }

  /**
   * @brief Gets new index for tree expansion
   * @param target_index Tree target index
   * @return unsigned int
   */
  unsigned int operator()(const unsigned int& target_index);

 private:
  /**
   * @brief RGD index generation
   * @param random_index
   * @param target_index
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
  // Generator parameters
  GeneratorParams params_;
  // Utils for random number generation
  std::minstd_rand gen_;
  std::uniform_real_distribution<double> dist_;
};

}  // namespace rrt_planner

#endif
