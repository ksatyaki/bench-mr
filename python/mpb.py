import random
import subprocess
import datetime
import time
import itertools
import os
import sys
import psutil
import resource
import json
from typing import Optional, Union
from threading import Timer
from copy import deepcopy

from utils import *
from multiprocessing import Pool
from tqdm.notebook import tqdm

MPB_BINARY = './benchmark'
MPB_BINARY_DIR = '../bin'

# limit memory by this fraction of available memory if activated for parallel MPB execution
MEMORY_LIMIT_FRACTION = min(0.9, 5. / os.cpu_count())


class MPB:
    def __init__(self,
                 config_file: str = os.path.join(MPB_BINARY_DIR, 'benchmark_template.json'),
                 output_path: str = ''):
        bin_path = os.path.abspath(os.path.join(MPB_BINARY_DIR, MPB_BINARY))
        if not os.path.exists(bin_path):
            raise Exception('Error: Could not find benchmark binary file at %s. ' % bin_path +
                            'Make sure you have built it and set the correct MPB_BINARY_DIR variable in %s.' % __file__)
        self.config = MPB.get_config(config_file)  # type: dict
        # if not config_file.endswith("benchmark_template.json"):
        #     print("Created MPB from config %s." % config_file)
        self.output_path = output_path  # type: str
        self.id = None  # type: Optional[str]
        self._update_pss()

        # print("planners:       \t", self._planners)
        # print("smoothers:      \t", self._smoothers)
        # print("steer_functions:\t", self._steer_functions)
        # self.set_planners(['rrt'])
        # self.set_smoothers([])
        # self.set_steer_functions(['reeds_shepp'])
        self.config_filename = config_file  # type: Optional[str]
        self.results_filename = None  # type: Optional[str]

    def __getitem__(self, item: str) -> Union[str, int, float, dict]:
        c = self.config
        for s in item.split('.'):
            c = c[s]
        return c

    def __setitem__(self, item: str, value: object) -> object:
        c = self.config
        splits = item.split('.')
        for s in splits[:-1]:
            c = c[s]
        c[splits[-1]] = value
        self._update_pss()
        return value

    def update(self, config: dict) -> dict:
        for key, value in config.items():
            self[key] = value
        self._update_pss()
        return self.config

    def _update_pss(self):
        """
        Update planners, smoothers, steer functions from config.
        """
        self._planners = [planner for planner, used in self["benchmark.planning"].items(
        ) if used]  # type: [str]
        self._smoothers = [smoother for smoother,
                           used in self["benchmark.smoothing"].items() if used]  # type: [str]
        self._steer_functions = [steer_functions[index]
                                 for index in self["benchmark.steer_functions"]]  # type: [str]
        self._robot_models = [robot_models[index]
                              for index in self["benchmark.forward_propagations"]]

    @staticmethod
    def get_config(config_file: str = os.path.join(MPB_BINARY_DIR, 'benchmark_template.json')) -> dict:
        if not os.path.exists(config_file):
            raise Exception('Could not find configuration template file at %s. ' % os.path.abspath(
                os.path.join(MPB_BINARY_DIR, 'benchmark_template.json')) +
                'Make sure you run the benchmark binary without CLI arguments to generate ' +
                'benchmark_template.json.')
        with open(config_file, 'r') as f:
            config = json.load(f)["settings"]  # type: dict
        return config

    def set_random_grid_env(self,
                            width: int = 50,
                            height: int = 50,
                            obstacle_ratio: float = 0.1,
                            seed: int = 1):
        self["env.type"] = "grid"
        self["env.grid.generator"] = "random"
        self["env.grid.width"] = width
        self["env.grid.height"] = height
        self["env.grid.seed"] = seed
        self["env.grid.random.obstacle_ratio"] = obstacle_ratio

    def set_corridor_grid_env(self,
                              width: int = 50,
                              height: int = 50,
                              branches: int = 40,
                              radius: float = 3.,
                              seed: int = 1):
        self["env.type"] = "grid"
        self["env.grid.generator"] = "corridor"
        self["env.grid.width"] = width
        self["env.grid.height"] = height
        self["env.grid.seed"] = seed
        self["env.grid.corridor.branches"] = branches
        self["env.grid.corridor.radius"] = radius

    def set_image_grid_env(self,
                           filename: str,
                           desired_width: int = 0,
                           desired_height: int = 0,
                           occupancy_threshold: float = 0.5):
        self["env.type"] = "grid"
        self["env.grid.generator"] = "image"
        self["env.grid.image.source"] = filename
        self["env.grid.image.desired_width"] = desired_width
        self["env.grid.image.desired_height"] = desired_height
        self["env.grid.image.occupancy_threshold"] = occupancy_threshold
        
    def set_image_yaml_env(self,
                           filename: str):
        self["env.type"] = "yaml"
        self["env.grid.image.source"] = filename

    def set_polygon_env(self,
                        filename: str,
                        scaling: float = 1.):
        self["env.type"] = "polygon"
        self["env.polygon.source"] = filename
        self["env.polygon.scaling"] = scaling

    def set_start(self,
                  x: float,
                  y: float,
                  theta: float):
        self["env.start.x"] = x
        self["env.start.y"] = y
        self["env.start.theta"] = theta

    def set_goal(self,
                 x: float,
                 y: float,
                 theta: float):
        self["env.goal.x"] = x
        self["env.goal.y"] = y
        self["env.goal.theta"] = theta

    def set_planners(self, planners: [str]):
        planners = list(
            set(itertools.chain.from_iterable(map(parse_planners, planners))))
        self._planners = []
        for p in self["benchmark.planning"].keys():
            if p in planners:
                self["benchmark.planning." + p] = True
                self._planners.append(p)
            else:
                self["benchmark.planning." + p] = False
        self._planners = list(set(self._planners))
        if len(planners) != len(self._planners):
            print("Error: Some planner could not be unified. From the given planners", planners,
                  "the following were selected:", self._planners,
                  file=sys.stderr)

    def set_steer_functions(self, steerings: [str]):
        self._steer_functions = list(
            set(itertools.chain.from_iterable(map(parse_steer_functions, steerings))))
        self["benchmark.steer_functions"] = self._steer_functions
        self["benchmark.control_planners_on"] = False
        if len(steerings) != len(self._steer_functions):
            print("Error: Some steer function could not be unified. Selected steer functions:",
                  [steer_functions[index] for index in self._steer_functions],
                  file=sys.stderr)

    def set_robot_models_functions(self, robot_models: [str]):
        self._robot_models = list(
            set(itertools.chain.from_iterable(map(parse_robot_models, robot_models))))
        self["benchmark.forward_propagations"] = self._robot_models
        self["benchmark.control_planners_on"] = True
        if len(robot_models) != len(self._robot_models):
            print("Error: Some robot models could not be unified. Selected robot models:",
                  [robot_models[index] for index in self._robot_models],
                  file=sys.stderr)

    def set_exact_goal_radius(self, radius: float = 0.1):
        print(radius)
        self["exact_goal_radius"] = radius

    def set_smoothers(self, smoothers: [str]):
        self._smoothers = []
        for p in self["benchmark.smoothing"].keys():
            if p in smoothers:
                self["benchmark.smoothing." + p] = True
                self._smoothers.append(p)
            else:
                self["benchmark.smoothing." + p] = False
        self._smoothers = list(set(self._smoothers))
        if len(smoothers) != len(self._smoothers):
            print("Error: Some smoother could not be unified. Selected smoothers:", self._smoothers,
                  file=sys.stderr)

    def save_settings(self, filename: str):
        with open(filename, 'w') as f:
            json.dump({
                "settings": self.config
            }, f, indent=2)

    def set_id(self, id: str):
        self.id = id

    def set_subfolder(self, subfolder: str = ''):
        self.config_filename = os.path.join(
            subfolder, self.id) + "_config.json"
        self.results_filename = os.path.join(
            subfolder, self.id) + "_results.json"
        self["benchmark.log_file"] = os.path.abspath(self.results_filename)

    @staticmethod
    def get_memory():
        """
        Retrieves available system memory.
        """
        # thanks to https://stackoverflow.com/a/41125461
        with open('/proc/meminfo', 'r') as mem:
            free_memory = 0
            for i in mem:
                sline = i.split()
                if str(sline[0]) in ('MemFree:', 'Buffers:', 'Cached:'):
                    free_memory += int(sline[1])
        return free_memory

    def run(self, id: str = None, runs: Optional[int] = None, subfolder: str = '',
            show_progress_bar: bool = True, shuffle_planners: bool = True,
            kill_after_timeout: bool = True, silence: bool = False) -> int:
        if runs:
            self["benchmark.runs"] = runs
        else:
            runs = self["benchmark.runs"]
        ts = time.time()
        if not id:
            id = datetime.datetime.fromtimestamp(
                ts).strftime('%Y-%m-%d_%H-%M-%S')
        self.set_id(id)
        self.set_subfolder(subfolder)
        log_filename = os.path.join(subfolder, self.id + ".log")
        logfile = open(log_filename, 'w+')
        if not silence:
            print("Running MPB with ID %s (log file at %s)..." %
                  (self.id, log_filename))
        num_planners = len(self._planners)
        total_iterations = num_planners * len(self._steer_functions) * runs
        if show_progress_bar:
            pbar = tqdm(range(total_iterations), desc=self.id) #, ncols='100%')
        success = True
        code = 0
        results_filenames = []
        if shuffle_planners:
            # shuffle planners to avoid multiple parallel MPBs run the same heavy-load planners
            # (e.g. CForest takes all available threads, SBPL leaks memory) at the same time
            random.shuffle(self._planners)
        for ip, planner in enumerate(self._planners):
            run = 0

            def pbar_prompt():
                if not show_progress_bar:
                    return
                if runs > 1:
                    pbar.display('%s (%i / %i) [run %i / %i]' % (
                        convert_planner_name(planner), ip + 1, num_planners, min(run + 1, runs), runs))
                else:
                    pbar.display(
                        '%s (%i / %i)' % (convert_planner_name(planner), ip + 1, num_planners))

            pbar_prompt()

            if ip == 0:
                results_filename = self.results_filename
            else:
                results_filename = os.path.join(
                    subfolder, self.id + "_results_%s.json" % planner)
            self["benchmark.log_file"] = os.path.abspath(results_filename)
            for p in self["benchmark.planning"].keys():
                self["benchmark.planning." + p] = p == planner
            self.save_settings(self.config_filename)
            tsk = subprocess.Popen([MPB_BINARY, os.path.abspath(self.config_filename)],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   cwd=os.path.abspath(MPB_BINARY_DIR))
            proc = psutil.Process(tsk.pid)
            create_time = time.time()
            kill_timer = None
            if kill_after_timeout:
                # kill process after 2 * max planning time
                def kill_process():
                    nonlocal success, code
                    try:
                        proc.kill()
                        print("Killed %s with planner %s after %.2fs exceeded timeout."
                              % (self.id, planner, time.time() - create_time))
                        success = False
                        code = -9
                    except psutil.NoSuchProcess:
                        pass
                    except:
                        print('Error occurred while trying to kill %s with planner %s.'
                              % (self.id, planner), file=sys.stderr)
                        success = False
                        code = -9

                kill_timer = Timer(
                    self["max_planning_time"] * self["benchmark.runs"] * 2, kill_process)
                kill_timer.start()
            while True:
                line = tsk.stdout.readline()
                if line is None:
                    break
                line = line.decode('UTF-8')
                if line == '':
                    break
                if '<stats>' in line:
                    run += 1
                    # some planner (and its smoothers) has finished
                    if show_progress_bar:
                        pbar.update(1)
                        pbar_prompt()
                logfile.write(line)
            code = tsk.poll()
            if code is None:
                code = 0
            if code is not None and code != 0:
                print("Error (%i) occurred for MPB with ID %s using planner %s." % (
                    code, self.id, convert_planner_name(planner)),
                    file=sys.stderr)
                success = False
                continue
            if ip > 0:
                results_filenames.append(results_filename)
                MPB.merge([self.results_filename, results_filename],
                          self.results_filename, silence=True)
            # if show_progress_bar:
            #     pbar.update(1)
            if kill_timer is not None:
                kill_timer.cancel()
        if show_progress_bar:
            pbar.close()
        logfile.close()
        # remove partial results files
        for results_filename in results_filenames:
            try:
                os.remove(results_filename)
            except:
                print("Error: results %s do not exist." %
                      results_filename, file=sys.stderr)
        return code

    def print_info(self):
        if not os.path.exists(self.results_filename):
            print("No results file exists for MPB %s." % self.id)
            return
        with open(self.results_filename, "r") as f:
            data = json.load(f)
            run_ids = list(range(len(data["runs"])))
            for run_id in run_ids:
                print_run_info(data, run_id, run_ids)

    def visualize_trajectories(self, **kwargs):
        if not os.path.exists(self.results_filename):
            print("No results file exists for MPB %s." % self.id)
            return
        from trajectory import visualize
        visualize(self.results_filename, **kwargs)

    def visualize_trajectory_grid(self, set_suptitle=True, **kwargs):
        if not os.path.exists(self.results_filename):
            print("No results file exists for MPB %s." % self.id)
            return
        from trajectory import visualize_grid
        if set_suptitle:
            kwargs["suptitle"] = self.id
        visualize_grid(self.results_filename,  **kwargs)

    def plot_planner_stats(self, **kwargs):
        if not os.path.exists(self.results_filename):
            print("No results file exists for MPB %s." % self.id)
            return
        from plot_stats import plot_planner_stats
        plot_planner_stats(self.results_filename, **kwargs)

    def plot_smoother_stats(self, **kwargs):
        if not os.path.exists(self.results_filename):
            print("No results file exists for MPB %s." % self.id)
            return
        from plot_stats import plot_smoother_stats
        plot_smoother_stats(self.results_filename, **kwargs)

    def plot_planner_timings(self, **kwargs):
        import matplotlib.pyplot as plt
        import json
        import numpy as np
        data = json.load(open(self.results_filename, "r"))
        for run_id in range(len(data["runs"])):
            plt.figure("Run %i" % run_id)
            planners, total_times, steering_times, collision_times = [], [], [], []
            for planner, plan in data["runs"][run_id]["plans"].items():
                planners.append(planner)
                total_times.append(plan["stats"]["planning_time"])
                steering_times.append(plan["stats"]["steering_time"])
                collision_times.append(plan["stats"]["collision_time"])

            xs = np.arange(len(planners)) + 0.5
            plt.title("Run %i" % run_id)
            plt.bar(xs, total_times, width=0.75, edgecolor="black",
                    linewidth=2, linestyle="-", label="Total time")
            plt.bar(xs, np.array(collision_times) +
                    np.array(steering_times), width=0.7, label="Steering")
            plt.bar(xs, collision_times, width=0.7, label="Collision")
            plt.legend()
            plt.xticks(xs, [convert_planner_name(p)
                            for p in planners], rotation=0, fontsize=14)
            plt.gca().set_xlim([0, len(planners)])
            plt.show()

    @staticmethod
    def merge(mpbs, target_filename: str, make_separate_runs: bool = False, silence: bool = False,
              plan_names: [str] = None):
        """
        Merges results of the given MPB instances into one file.
        """
        results_filenames = []
        for i, m in enumerate(mpbs):
            if 'mpb.MPB' in str(type(m)):
                results_filenames.append(m.results_filename)
            elif type(m) == str:
                results_filenames.append(m)
            else:
                raise TypeError(
                    "MPB.merge requires list of MPB instances or filenames.")

        plan_index = 0
        target = None
        for i, m in enumerate(mpbs):
            if results_filenames[i] is None:
                print("No results file exists for MPB %s. Skipping." % str(m))
                continue
            if not os.path.exists(results_filenames[i]):
                if not silence:
                    print("No results file exists for MPB %s. Skipping." % str(m))
                if 'mpb.MPB' in str(type(m)):
                    plan_index += len(m._planners)
                continue
            with open(results_filenames[i]) as res_file:
                try:
                    res = json.load(res_file)
                    if res is None or "runs" not in res:
                        print("Benchmark %s has empty results file %s. Skipping."
                              % (str(m), results_filenames[i]), file=sys.stderr)
                        continue
                    if i == 0 or target is None:
                        target = deepcopy(res)
                        target["runs"] = []
                    if target is None:
                        continue

                    # TODO check settings, environments are the same for each run before merging
                    for run_id, run in enumerate(res["runs"]):
                        if make_separate_runs:
                            target["runs"].append(run)
                            continue
                        if i == 0:
                            target["runs"].append(deepcopy(run))
                            target["runs"][run_id]["plans"] = {}

                        if run_id >= len(target["runs"]) and i != 0:
                            if not silence:
                                print("Run #%i does not exist in %s but in %s. Skipping."
                                      % (run_id, results_filenames[i - 1], results_filenames[i]), file=sys.stderr)
                        else:
                            if "plans" not in run or run["plans"] is None:
                                continue
                            for pi, (planner, plan) in enumerate(run["plans"].items()):
                                if plan_names:
                                    target["runs"][run_id]["plans"][plan_names[plan_index + pi]] = plan
                                    continue
                                if planner in target["runs"][run_id]["plans"]:
                                    if not silence:
                                        print("Planner %s already exists in %s and in %s. Skipping."
                                              % (planner, results_filenames[i - 1], results_filenames[i]),
                                              file=sys.stderr)
                                else:
                                    target["runs"][run_id]["plans"][planner] = plan
                except json.decoder.JSONDecodeError:
                    print("Error while decoding JSON file %s." %
                          results_filenames[i], file=sys.stderr)

            if 'mpb.MPB' in str(type(m)):
                plan_index += len(m._planners)

        with open(target_filename, "w") as target_file:
            json.dump(target, target_file, indent=2)
            if not silence:
                print("Successfully merged [%s] into %s." % (
                    ", ".join(results_filenames), target_filename))
        m = MPB()
        m.results_filename = target_filename
        m.set_id(os.path.basename(os.path.splitext(target_filename)[0]))
        return m


class MultipleMPB:
    def __init__(self):
        self.id = None  # type: Optional[str]
        self.benchmarks = []  # type: [MPB]
        self.subfolder = ''

    def __getitem__(self, item: str, index: int = 0) -> dict:
        return self.benchmarks[index][item]

    def __setitem__(self, item: str, value):
        for i in range(len(self.benchmarks)):
            self.benchmarks[i][item] = value

    def update(self, config: dict):
        for m in self.benchmarks:
            m.update(config)

    def merge(self, *args, **kwargs):
        MPB.merge(self.benchmarks, *args, **kwargs)

    @staticmethod
    def run_(arg) -> int:
        config_filename, index, mpb_id, subfolder, memory_limit, runs, silence = arg
        if memory_limit != 0:
            resource.setrlimit(resource.RLIMIT_AS, memory_limit)
        mpb = MPB(config_file=config_filename)
        code = mpb.run(id=mpb_id,
                       runs=runs,
                       subfolder=subfolder,
                       show_progress_bar=not silence,
                       silence=silence)
        if not silence:
            if code == 0:
                print("Benchmark %i (%s) finished successfully." %
                      (index, mpb_id))
            elif code is not None:
                print("Benchmark %i (%s) failed. Return code: %i." %
                      (index, mpb_id, code), file=sys.stderr)
            else:
                print("Benchmark %i (%s) failed. Unknown return code." %
                      (index, mpb_id), file=sys.stderr)
        return code

    def run_parallel(self,
                     id: str = None,
                     use_subfolder: bool = True,
                     runs: int = 1,
                     silence: bool = False,
                     processes: int = os.cpu_count(),
                     limit_memory: bool = True,
                     show_plot: bool = True) -> bool:
        memory_limit = 0
        if limit_memory:
            print("Available memory: %.2f GB, limiting each MPB process to %.1f%% usage (%.2f GB)." %
                  (MPB.get_memory() / 1e6, MEMORY_LIMIT_FRACTION * 100, MPB.get_memory() / 1e6 * MEMORY_LIMIT_FRACTION))
            soft, hard = resource.getrlimit(resource.RLIMIT_AS)
            memory_limit = (MPB.get_memory() * 1024 *
                            MEMORY_LIMIT_FRACTION, hard)

        self["benchmark.runs"] = runs
        ts = time.time()
        if id:
            self.id = id
        else:
            self.id = datetime.datetime.fromtimestamp(
                ts).strftime('%Y-%m-%d_%H-%M-%S')
        if use_subfolder:
            if not os.path.exists(self.id):
                os.mkdir(self.id)
            self.subfolder = self.id

        ids = []
        config_files = []
        log_files = []
        subfolder_id = os.path.join(self.subfolder, self.id)
        for i, mpb in enumerate(self.benchmarks):
            if mpb.id is None:
                filename = "%s_%i_config.json" % (subfolder_id, i)
                mpb.set_id("%s_%i" % (self.id, i))
                log_files.append("%s_%i.log" % (subfolder_id, i))
            else:
                filename = os.path.join(
                    self.subfolder, mpb.id + "_config.json")
                log_files.append(os.path.join(self.subfolder, mpb.id + ".log"))
            mpb.save_settings(filename)
            config_files.append(filename)
            ids.append(mpb.id)
            mpb.set_subfolder(self.id if use_subfolder else '')
        processes = min(processes, len(self.benchmarks))
        print("Creating pool of %i processes." % processes)
        sys.stdout.flush()
        with Pool(processes) as pool:
            results = pool.map(MultipleMPB.run_,
                               zip(config_files,
                                   range(len(self.benchmarks)),
                                   ids,
                                   [self.subfolder] * len(self.benchmarks),
                                   [memory_limit] * len(self.benchmarks),
                                   [runs] * len(self.benchmarks),
                                   [silence] * len(self.benchmarks)))

            if show_plot:
                try:
                    import matplotlib.pyplot as plt
                    from plot_aggregate import plot_aggregate_stats
                    from utils import get_aggregate_stats
                    f, (a0, a1) = plt.subplots(1, 2, gridspec_kw={
                        'width_ratios': [1, 4]}, figsize=(10, 3))
                    plt.title(self.id, loc="left", fontweight="bold")
                    plt.title(str(datetime.datetime.now()), loc="right")
                    counts = {}
                    known_codes = {
                        -9: "timeout",
                        0: "success"
                    }
                    for code in results:
                        if code is None:
                            code_name = "unknown error"
                        else:
                            code_name = known_codes.get(
                                code, "error %d" % code)
                        counts[code_name] = counts.get(code_name, 0) + 1
                    total = sum(counts.values())
                    a0.pie(list(counts.values()), labels=list(counts.keys()),
                           autopct=lambda p: '{:.0f}'.format(p * total / 100))

                    aggregate = get_aggregate_stats(
                        [m.results_filename for m in self.benchmarks])
                    plot_aggregate_stats(a1,
                                         aggregate["total"],
                                         aggregate["found"],
                                         aggregate["collision_free"],
                                         aggregate["exact"],
                                         show_aggregate_title=False)
                    plt.tight_layout()
                    plt.subplots_adjust(0., 0.1, 1, 0.9, 0.3, 0.4)
                except Exception as e:
                    print(
                        "Error while plotting benchmark progress overview:", e, file=sys.stderr)

            if all([r == 0 for r in results]):
                print("All benchmarks succeeded.")
            else:
                print("Error(s) occurred, not all benchmarks succeeded.",
                      file=sys.stderr)
                for i, code in enumerate(results):
                    if code == 0:
                        continue
                    elif code is None:
                        print("Benchmark %i failed with unknown return code. See log file %s."
                              % (i, log_files[i]), file=sys.stderr)
                    else:
                        print("Benchmark %i failed with return code %i. See log file %s."
                              % (i, code, log_files[i]), file=sys.stderr)
                return False
        return True

    def visualize_trajectories(self, **kwargs):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(6 * len(self.benchmarks), 6))
        for i, m in enumerate(self.benchmarks):
            plt.subplot(1, len(self.benchmarks), i + 1)
            if "headless" not in kwargs:
                kwargs["headless"] = True
            if "combine_views" not in kwargs:
                kwargs["combine_views"] = False
            if "use_existing_subplot" not in kwargs:
                kwargs["use_existing_subplot"] = True
            if "show_legend" not in kwargs:
                kwargs["show_legend"] = False
            if "silence" not in kwargs:
                kwargs["silence"] = True
            m.visualize_trajectories(**kwargs)
            plt.title("%s" % m.id)
        plt.tight_layout()

    def visualize_trajectory_grid(self, merge_file_name: str = None, set_suptitle=True, **kwargs):
        if merge_file_name is None:
            merge_file_name = "%s_total.json" % self.id
        self.merge(target_filename=merge_file_name, make_separate_runs=True)
        from trajectory import visualize_grid
        if set_suptitle:
            kwargs["suptitle"] = self.id
        visualize_grid(merge_file_name, **kwargs)

    def plot_planner_stats(self, **kwargs):
        import matplotlib.pyplot as plt
        for m in self.benchmarks:
            m.plot_planner_stats(**kwargs)
            plt.suptitle(m.id, fontsize=24, y=1.05)
            plt.tight_layout()

    def plot_smoother_stats(self, **kwargs):
        import matplotlib.pyplot as plt
        for i, m in enumerate(self.benchmarks):
            m.plot_smoother_stats(**kwargs)
            plt.suptitle(m.id, fontsize=24, y=1.05)
            plt.tight_layout()

    def get_all_planners(self) -> [str]:
        planners = []
        for m in self.benchmarks:
            if m.results_filename is None or not os.path.exists(m.results_filename):
                continue
            planners += get_planners(m.results_filename)
        planners = list(set(planners))
        planners = list(sorted(planners, key=convert_planner_name))
        return planners
