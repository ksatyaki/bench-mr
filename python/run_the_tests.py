import yaml
from mpb import MPB, MultipleMPB
import matplotlib as mpl
mpl.rcParams['mathtext.fontset'] = 'cm'
mpl.rcParams['pdf.fonttype'] = 42

from copy import deepcopy
import math

from argparse import ArgumentParser
from yaml.loader import SafeLoader
from tqdm import tqdm


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("setup_yaml_file",
                        help="The data file where all trajectory data is stored. Required argument.",
                        metavar="DATAFILE"
                        )
    args = parser.parse_args()

    with open(args.setup_yaml_file) as f:
        setup = yaml.load(f, SafeLoader)

    tqdm.write("Successfully read the yaml file containing {} start-goal pairs.", len(setup['sg']))

    # Basic Setup
    mpb = MPB()
    mpb["ompl.seed"] = -1  # set the seed of the OMPL planners
    mpb.set_planners(['rrt_star'])
    mpb.set_steer_functions(['car'])
    mpb["steer.car_turning_radius"] = 1.0
    mpb["steer.sampling_resolution"] = 0.01
    mpb["max_planning_time"] = 900
    mpb["ompl.geometric_planner_settings.RRTstar.delay_collision_checking"] = "0"
    mpb["ompl.geometric_planner_settings.RRTstar.goal_bias"] = "0.05"
    # mpb["ompl.geometric_planner_settings.RRTstar.informed_sampling"] = "1"

    folder_prefix = "/home/ksatyaki/workspace/bench_ws/src/bench-mr/"
    mpb["env.collision.robot_shape_source"] = folder_prefix + "maps/simple_robot.yaml"
    mpb.set_image_yaml_env(folder_prefix + "maps/atc.yaml")
    cliff_map_file = folder_prefix + "maps/atc_cliff.xml"
    intensity_map_file = folder_prefix + "maps/atc_intensity1m.xml"
    gmmt_map_file = folder_prefix + "maps/atc_gmmt.xml"
    cost_fns = ["cliff", "intensity", "dtc", "gmmt"]
    cost_fn_map = {"dtc": cliff_map_file, "cliff": cliff_map_file, "intensity": intensity_map_file,
                   "gmmt": gmmt_map_file}

    mpb["mod.weight_dtc"] = 0.02
    mpb["mod.weight_cliff"] = 0.1
    mpb["mod.weight_gmmt"] = 0.1
    mpb["mod.weight_intensity"] = 0.2
    mpb.set_start(-5.0, -5.0, math.pi / 4.0)
    mpb.set_goal(19.0, 19.0, math.pi / 4.0)

    mpbs = dict()
    result_file_names = []
    for sgs in setup['sg']:
        mpb.set_start(sgs["start"])
        mpb.set_goal(sgs["goal"])
        results_folder_prefix = folder_prefix + (sgs["name"])

        for cost_fn in cost_fns:
            intensity_mpb = deepcopy(mpb)
            intensity_mpb["ompl.sampler"] = "intensity"
            intensity_mpb.set_id('{}-{}'.format(cost_fn, 'intensity'))
            intensity_mpb["mod.sampling_bias"] = 0.5
            intensity_mpb["ompl.intensity_map_file_name"] = intensity_map_file
            intensity_mpb["ompl.optimization_objective"] = cost_fn
            intensity_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
            mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'intensity')] = intensity_mpb
            result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'intensity'))

            ellipse_mpb = deepcopy(mpb)
            ellipse_mpb.set_id('{}-{}'.format(cost_fn, 'ellipse'))
            ellipse_mpb["ompl.intensity_map_file_name"] = intensity_map_file
            ellipse_mpb["ompl.optimization_objective"] = cost_fn
            ellipse_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
            ellipse_mpb.set_planners(['informed_rrt_star'])
            mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'ellipse')] = ellipse_mpb
            result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'ellipse'))

            dijkstra_mpb = deepcopy(mpb)
            dijkstra_mpb["ompl.sampler"] = "dijkstra"
            dijkstra_mpb["mod.dijkstra_cell_size"] = 0.5
            dijkstra_mpb["mod.sampling_bias"] = 0.05
            dijkstra_mpb.set_id('{}-{}'.format(cost_fn, 'dijkstra'))
            dijkstra_mpb["ompl.intensity_map_file_name"] = intensity_map_file
            dijkstra_mpb["ompl.optimization_objective"] = cost_fn
            dijkstra_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
            mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'dijkstra')] = dijkstra_mpb
            result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'dijkstra'))

            uniform_mpb = deepcopy(mpb)
            uniform_mpb["ompl.sampler"] = "iid"
            uniform_mpb.set_id('{}-{}'.format(cost_fn, 'uniform'))
            uniform_mpb["ompl.intensity_map_file_name"] = intensity_map_file
            uniform_mpb["ompl.optimization_objective"] = cost_fn
            uniform_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
            mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'uniform')] = uniform_mpb
            result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'uniform'))

    for key in mpbs:
        mpbs[key].run(id=key, runs=20, subfolder=folder_prefix + "/python/{}".format(results_folder_prefix))


