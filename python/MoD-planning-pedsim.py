from mpb import MPB, MultipleMPB
import matplotlib as mpl

mpl.rcParams['mathtext.fontset'] = 'cm'
# make sure to not use Level-3 fonts
mpl.rcParams['pdf.fonttype'] = 42

from copy import deepcopy
import math

mpb = MPB()
mpb["ompl.seed"] = -1  # set the seed of the OMPL planners
mpb.set_planners(['rrt_star'])
mpb.set_steer_functions(['car'])
mpb["steer.car_turning_radius"] = 1.0
mpb["steer.sampling_resolution"] = 0.01
mpb["max_planning_time"] = 8
mpb["ompl.geometric_planner_settings.RRTstar.delay_collision_checking"] = "0"
mpb["ompl.geometric_planner_settings.RRTstar.goal_bias"] = "0.05"
mpb["ompl.geometric_planner_settings.RRTstar.informed_sampling"] = "1"

folder_prefix = "/home/ksatyaki/workspace/bench_ws/src/bench-mr/"
mpb["env.collision.robot_shape_source"] = folder_prefix + "maps/simple_robot.yaml"
mpb.set_image_yaml_env(folder_prefix + "maps/office_cubicles.yaml")
cliff_map_file = folder_prefix + "maps/office_cubicles_cliffmap.xml"
intensity_map_file = folder_prefix + "maps/office_cubicles_intensitymap.xml"
gmmt_map_file = folder_prefix + "maps/office_cubicles_gmmtmap.xml"

cost_fns = ["cliff", "intensity", "dtc", "gmmt"]
cost_fn_map = {"dtc": cliff_map_file, "cliff": cliff_map_file, "intensity": intensity_map_file, "gmmt": gmmt_map_file}

mpb["mod.weight_dtc"] = 0.05
mpb["mod.weight_cliff"] = 0.25
mpb["mod.weight_gmmt"] = 0.25
mpb["mod.weight_intensity"] = 0.50
mpb.set_start(-5.0, -5.0, math.pi / 4.0)
mpb.set_goal(19.0, 19.0, math.pi / 4.0)

results_folder_prefix = "mod-tests-1"

mpbs = dict()
result_file_names = []
for cost_fn in cost_fns:
    intensity_mpb = deepcopy(mpb)
    intensity_mpb["ompl.sampler"] = "intensity"
    intensity_mpb.set_id('{}-{}'.format(cost_fn, 'intensity'))
    intensity_mpb["mod.sampling_bias"] = 1.0
    intensity_mpb["ompl.intensity_map_file_name"] = intensity_map_file
    intensity_mpb["ompl.optimization_objective"] = cost_fn
    intensity_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
    mpbs['{}-{}'.format(cost_fn, 'intensity')] = intensity_mpb
    result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'intensity'))

    ellipse_mpb = deepcopy(mpb)
    ellipse_mpb.set_id('{}-{}'.format(cost_fn, 'ellipse'))
    ellipse_mpb["ompl.intensity_map_file_name"] = intensity_map_file
    ellipse_mpb["ompl.optimization_objective"] = cost_fn
    ellipse_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
    ellipse_mpb.set_planners(['informed_rrt_star'])
    mpbs['{}-{}'.format(cost_fn, 'ellipse')] = ellipse_mpb
    result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'ellipse'))

    dijkstra_mpb = deepcopy(mpb)
    dijkstra_mpb["ompl.sampler"] = "dijkstra"
    dijkstra_mpb["mod.dijkstra_cell_size"] = 0.5
    dijkstra_mpb["mod.sampling_bias"] = 0.05
    dijkstra_mpb.set_id('{}-{}'.format(cost_fn, 'dijkstra'))
    dijkstra_mpb["ompl.intensity_map_file_name"] = intensity_map_file
    dijkstra_mpb["ompl.optimization_objective"] = cost_fn
    dijkstra_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
    mpbs['{}-{}'.format(cost_fn, 'dijkstra')] = dijkstra_mpb
    result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'dijkstra'))

    uniform_mpb = deepcopy(mpb)
    uniform_mpb["ompl.sampler"] = "iid"
    uniform_mpb.set_id('{}-{}'.format(cost_fn, 'uniform'))
    uniform_mpb["ompl.intensity_map_file_name"] = intensity_map_file
    uniform_mpb["ompl.optimization_objective"] = cost_fn
    uniform_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
    mpbs['{}-{}'.format(cost_fn, 'uniform')] = uniform_mpb
    result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'uniform'))

for key in mpbs:
    mpbs[key].run(id=key, runs=20, subfolder=folder_prefix + "/python/{}".format(results_folder_prefix))

MPB.merge(result_file_names, target_filename="{}/combined.json".format(results_folder_prefix, results_folder_prefix),
          plan_names=list(mpbs.keys()))
