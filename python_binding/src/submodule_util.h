#ifndef PYTHON_BINDING_SRC_SUBMODULE_UTIL_H_
#define PYTHON_BINDING_SRC_SUBMODULE_UTIL_H_

inline int preprocess_input_tvobstacles(
    py::list& obstacles_in,
    aligned_vector<const collision::TimeVariantCollisionObject*>&
        obstacles_out) {
  for (auto el : obstacles_in)
    obstacles_out.push_back(
        el.cast<collision::TimeVariantCollisionObjectConstPtr>().get());
  return obstacles_out.size() == 0;
}

inline int postprocess_dynamic_collision_result(aligned_vector<int>& result,
                                                int num_trajectories,
                                                py::list& ret_list) {
  int num_colliding = 0;


  for (int cur_obst = 0; cur_obst < num_trajectories; cur_obst++) {
    ret_list.append(result[cur_obst]);
  }

  ret_list.append(*(result.end() - 2));
  ret_list.append(*(result.end() - 1));
  return 0;
}

#endif /* PYTHON_BINDING_SRC_SUBMODULE_UTIL_H_ */
