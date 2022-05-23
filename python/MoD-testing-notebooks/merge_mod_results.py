from mpb import MPB

import argparse

parser = argparse.ArgumentParser(
    description="Merge results files from the MPBs by specifying cost function and sampling function")
parser.add_argument("--cost-fn", nargs="*", type=str, help="A list of cost functions (space separated).")
parser.add_argument("--sampling-fn", nargs="*", type=str, help="A list of sampling functions (space separated).")
parser.add_argument("--folder", nargs="*", type=str,
                    help="The folder(s) where the source and destination result files are stored.")
args = parser.parse_args()

if not (args.cost_fn and args.sampling_fn and args.folder):
    parser.print_help()
    print("All three arguments are needed!")
    exit(-1)

for folder in args.folder:
    files = []
    planners = []
    for cost_fn in args.cost_fn:
        for sampling_fn in args.sampling_fn:
            files.append("{}/{}-{}_results.json".format(folder, cost_fn, sampling_fn))
            planners.append("{}-{}".format(cost_fn, sampling_fn))
    print("Running merge with files={}, target_filename={}/{}-combined.json, plan_names={}".format(files, folder,
                                                                                                   folder[folder.rfind(
                                                                                                       "/"):-1],
                                                                                                   planners))
    assert (len(files) == len(planners))
    MPB.merge(files, target_filename="{}/combined.json".format(folder),
              plan_names=planners)

print("DONE!")
exit(0)
