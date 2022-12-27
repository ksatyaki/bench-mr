import argparse
import json
from argparse import ArgumentParser

parser = argparse.ArgumentParser(description="Prints plan names in results file.")
parser.add_argument("results_file", nargs="*", type=str, help="Results file to check plan names.")
args = parser.parse_args()

res = json.load(open(args.results_file))
name_components = args.results_file.split("_")[-2]
name_components = name_components[name_components.rfind("/")+1:]

for run in res["runs"]:
    for pi, (planner, plan) in enumerate(run["plans"].items()):
        print("Changing {} to {}".format(planner, name_components))