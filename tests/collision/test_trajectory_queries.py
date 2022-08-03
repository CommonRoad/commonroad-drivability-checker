import commonroad_dc.collision.trajectory_queries.trajectory_queries as trajectory_queries
import commonroad_dc.pycrcc as pycrcc
import numpy as np
from random_object_creator import RandomObjectCreator


creator = RandomObjectCreator(-20, 20, -20, 20, 500, 500)


def get_gt_result(traj_list, dyn_obstacles):
    if len(traj_list) == 0: return []

    ret = []
    for traj in traj_list:
        collision_time = -1
        for i in range(traj.time_start_idx(), traj.time_end_idx() + 1):
            subj_obj = traj.obstacle_at_time(i)
            if subj_obj is None:
                continue
            for obst in dyn_obstacles:
                obj = obst.obstacle_at_time(i)
                if obj is None:
                    continue
                else:
                    res = obj.collide(subj_obj)
                    if res == True:
                        collision_time = i
                        break
            if collision_time != -1: break
        ret.append(collision_time)
    return ret


def verify_tolerance(obj1, obj2):
    tol_res = [True]
    try:
        tol_res = pycrcc.Util.experimental.tolerance_negative_query(obj1, obj2, False, 1e-10, 1e-6)
    except(Exception):
        pass

    return tol_res[0]


def verify_differences(res_1, res_2, traj_list, dyn_obstacles):
    def verify_timestep(traj, ind1, ind2, dyn_obstacles):
        if ind1 == -1:
            ind = ind2
        elif ind2 == -1:
            ind = ind1
        else:
            ind = min(ind1, ind2)

        obj1 = traj.obstacle_at_time(ind)
        if obj1 is None:
            print('sanity check failed: the trajectory has no object at the returned index')
            return False
        sg_obstacles = pycrcc.ShapeGroup()
        for dyn_obst in dyn_obstacles:
            obj2 = dyn_obst.obstacle_at_time(ind)
            if obj2 is not None:
                sg_obstacles.add_shape(obj2)
        tol_res = verify_tolerance(obj1, sg_obstacles)
        if tol_res == False:
            """
            plt.figure(figsize=(10, 10))
            draw_object(obj1, draw_params={'collision': {'facecolor': 'green'}})
            draw_object(sg_obstacles, draw_params={'collision': {'facecolor': 'red'}})

            plt.autoscale()
            plt.axis('equal')
            plt.show()
            """
            print('trajectory query failure')
            return False
        return True

    ind_diff = res_1 == res_2
    for ind, val in enumerate(ind_diff):
        if val == False:
            # check if the trajectory timestep is on the border of some dynamic obstacle
            traj_1 = traj_list[ind]

            if verify_timestep(traj_1, res_1[ind], res_2[ind], dyn_obstacles) == False:
                return False
    return True


def generate_random_trajectory():
    start_ts = np.random.choice(range(0, 10))
    tvobst = pycrcc.TimeVariantCollisionObject(start_ts)
    traj_len = np.random.choice(range(0, 4))
    for i in range(traj_len):
        random_obj = creator.create_random_aabb()
        tvobst.append_obstacle(random_obj)
    return tvobst


iter_max = 5000
iter = 0
while iter <= iter_max:
    iter = iter + 1
    traj_list = list()
    for i in range(10):
        traj = generate_random_trajectory()
        traj_list.append(traj)
    dyn_obstacles = list()
    for i in range(3):
        dyn_obstacles.append(generate_random_trajectory())

    res_grid = np.asarray(
        trajectory_queries.trajectories_collision_dynamic_obstacles(traj_list, dyn_obstacles, method='grid'))
    res_fcl = np.asarray(
        trajectory_queries.trajectories_collision_dynamic_obstacles(traj_list, dyn_obstacles, method='fcl'))
    res_box2d = np.asarray(
        trajectory_queries.trajectories_collision_dynamic_obstacles(traj_list, dyn_obstacles, method='box2d'))
    res_gt = np.asarray(get_gt_result(traj_list, dyn_obstacles))

    if not (np.array_equal(res_grid, res_fcl) and np.array_equal(res_grid, res_box2d)):
        verify_differences(res_grid, res_fcl, traj_list, dyn_obstacles)
        verify_differences(res_grid, res_box2d, traj_list, dyn_obstacles)

        print('differences between 3 methods')
    if not np.array_equal(res_fcl, res_gt):
        print('difference between fcl and gt')
        if verify_differences(res_fcl, res_gt, traj_list, dyn_obstacles) == False:
            incorrect = get_gt_result(traj_list, dyn_obstacles)
    if iter % 1000 == 0:
        print(iter)

    pass
