import json
import os
import pathlib as pl
from graphviz import Digraph
import argparse

file_name = 'systems.json'

# parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--file', help='file name')
args = parser.parse_args()
# if file name is not provided, find the file in current directory
if args.file:
    file_name = args.file

if not os.path.exists(file_name):
    print(f'File {file_name} does not exist!')
    exit(1)

# read the json file
f = open(file_name, 'r')
systems = json.load(f)
f.close()

dot = Digraph(comment='The Round Table')

# set font size
dot.attr(fontname='helvetica')
dot.attr(fontsize='40')
dot.attr(rankdir='LR')

engine_aware = []

for system in systems['sim_systems']:
    if system['engine_aware']:
        engine_aware.append(system)

for system in engine_aware:
    name = system['name'].replace('class uipc::backend::cuda::', '')
    deps = [dep.replace('class uipc::backend::cuda::', '') for dep in system['deps']]
    dot.node(name, shape='egg', color='#82B366', style='filled', fillcolor='#D5E8D4')
    for dep in deps:
        dot.edge(dep, name, color='#F08E81', arrowhead = 'diamond')

dot.node('SimEngine', shape='box', color='#82B366', style='filled', fillcolor='#D5E8D4')
for system in engine_aware:
    name = system['name'].replace('class uipc::backend::cuda::', '')
    dot.edge(name, 'SimEngine', color='#F08E81', arrowhead = 'diamond')
    
print(dot)