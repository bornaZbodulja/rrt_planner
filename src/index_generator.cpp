/**
 * @file index_generator.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/index_generator.h"

using namespace rrt_planner;

template <typename NodeT>
IndexGenerator<NodeT>::IndexGenerator(
    const CollisionCheckerPtr& collision_checker) {
  UpdateCollisionChecker(collision_checker);
  gen_ = std::minstd_rand(std::random_device{}());
  dist_ = std::uniform_real_distribution<double>(0.0, 1.0);
}

template <typename NodeT>
unsigned int IndexGenerator<NodeT>::operator()(
    const unsigned int& target_index) {
  const double r = dist_(gen_);

  if (r <= params_.target_bias) {
    return target_index;
  }

  return RGD(GenerateRandomIndex(params_.state_space_size), target_index);
}

template <typename NodeT>
unsigned int IndexGenerator<NodeT>::RGD(const unsigned int& random_index,
                                        const unsigned int& target_index) {
  auto rand_coords = NodeT::GetCoordinates(random_index);
  auto target_coords = NodeT::GetCoordinates(target_index);
  double distance;
  unsigned char cost;

  // TODO: Add param for number of iterations
  for (unsigned int i = 1; i < 10; i++) {
    cost =
        collision_checker_->GetCost(static_cast<unsigned int>(rand_coords.x),
                                    static_cast<unsigned int>(rand_coords.y));
    // TODO: Add param for this cost
    if (cost > 200) {
      NodeT::GetIndex(rand_coords);
    }
    distance = NodeT::CoordinatesDistance(rand_coords, target_coords);
    rand_coords.x += 1.0 * (target_coords.x - rand_coords.x) / distance;
    rand_coords.y += 1.0 * (target_coords.y - rand_coords.y) / distance;
  }

  return NodeT::GetIndex(rand_coords);
}

template <typename NodeT>
unsigned int IndexGenerator<NodeT>::GenerateRandomIndex(
    const unsigned int& state_space_size) const {
  return std::experimental::randint(static_cast<unsigned int>(0),
                                    state_space_size);
}

template class IndexGenerator<Node2D>;
template class IndexGenerator<NodeHybrid>;
