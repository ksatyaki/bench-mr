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

    print("***************************************************************")
    print("Successfully read the yaml file containing {} start-goal pairs.", len(setup['sg']))
    print("***************************************************************")

    # Basic Setup
    mpb = MPB()
    mpb["ompl.seed"] = -1  # set the seed of the OMPL planners
    mpb.set_planners(['rrt_star'])
    mpb.set_steer_functions(['car'])
    mpb["steer.car_turning_radius"] = 1.0
    mpb["steer.sampling_resolution"] = 0.01
    mpb["max_planning_time"] = 256
    mpb["ompl.geometric_planner_settings.RRTstar.delay_collision_checking"] = "0"
    mpb["ompl.geometric_planner_settings.RRTstar.goal_bias"] = "0.01"
    mpb["ompl.geometric_planner_settings.RRTstar.informed_sampling"] = "1"

    folder_prefix = "/home/ksatyaki/workspace/bench_ws/src/bench-mr/"
    mpb["env.collision.robot_shape_source"] = folder_prefix + "maps/simple_robot.yaml"
    mpb.set_image_yaml_env(folder_prefix + "maps/atc.yaml")
    cliff_map_file = folder_prefix + "maps/atc_cliff.xml"
    intensity_map_file = folder_prefix + "maps/atc_intensity1m.xml"
    gmmt_map_file = folder_prefix + "maps/atc_gmmt.xml"
    cost_fns = ["cliff", "intensity", "dtc", "gmmt"]
    cost_fn_map = {"dtc": cliff_map_file, "cliff": cliff_map_file, "intensity": intensity_map_file,
                   "gmmt": gmmt_map_file}

    mpb["mod.weight_dtc"] = 0.2
    mpb["mod.weight_cliff"] = 1.0
    mpb["mod.weight_gmmt"] = 1.0
    mpb["mod.weight_intensity"] = 2.0

    for sgs in setup['sg']:
        mpbs = dict()
        result_file_names = []

        mpb.set_start(sgs["start"][0],sgs["start"][1],sgs["start"][2])
        mpb.set_goal(sgs["goal"][0],sgs["goal"][1],sgs["goal"][2])
        results_folder_prefix = sgs["name"]

        for cost_fn in cost_fns:
            uniform_mpb = deepcopy(mpb)
            uniform_mpb["ompl.sampler"] = ""
            uniform_mpb.set_id('{}-{}'.format(cost_fn, 'uniform'))
            uniform_mpb["ompl.intensity_map_file_name"] = intensity_map_file
            uniform_mpb["ompl.optimization_objective"] = cost_fn
            uniform_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
            mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'uniform')] = uniform_mpb
            result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'uniform'))
            
            ellipse_mpb = deepcopy(mpb)
            uniform_mpb["ompl.sampler"] = "ellipse"
            ellipse_mpb.set_id('{}-{}'.format(cost_fn, 'ellipse'))
            ellipse_mpb["ompl.intensity_map_file_name"] = intensity_map_file
            ellipse_mpb["ompl.optimization_objective"] = cost_fn
            ellipse_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
            ellipse_mpb.set_planners(['informed_rrt_star'])
            mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'ellipse')] = ellipse_mpb
            result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'ellipse'))

            intensity_mpb = deepcopy(mpb)
            intensity_mpb["ompl.sampler"] = "intensity"
            intensity_mpb.set_id('{}-{}'.format(cost_fn, 'intensity'))
            intensity_mpb["mod.sampling_bias"] = 0.5
            intensity_mpb["ompl.intensity_map_file_name"] = intensity_map_file
            intensity_mpb["ompl.optimization_objective"] = cost_fn
            intensity_mpb["mod.mod_file_name"] = cost_fn_map[cost_fn]
            mpbs['{}-{}-{}'.format(sgs["name"], cost_fn, 'intensity')] = intensity_mpb
            result_file_names.append("{}/{}-{}_results.json".format(results_folder_prefix, cost_fn, 'intensity'))
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

        for key in mpbs:
            mpbs[key].run(id=key, runs=20, subfolder=folder_prefix + "/python/{}".format(results_folder_prefix))

        MPB.merge(result_file_names, target_filename="{}/combined.json".format(results_folder_prefix), plan_names=list(mpbs.keys()))



