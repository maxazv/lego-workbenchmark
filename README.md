# Lego Workbenchmark


## Setup

### TAMPanda

Install:
```bash
git clone https://github.com/snoato/TAMPanda.git
cd tampanda
pip install -e .
```
Now create symlink inside `src/`:
```bash
cd src
ln -s ../TAMPanda/tampanda .
```

### WorkBenchMark Dataset
In main repo folder (ie one folder level above src: `src\..`):
```bash
git clone https://github.com/WorkBenchMark/dataset.git
```
Now create symlink inside `src/`:
```bash
cd src
ln -s ../dataset .
```

### Lego Simulator
download (to do)
```bash
git clone https://github.com/ma-haha-hehe/lego_sim.git
```

### Fast-Downward
```bash
git clone https://github.com/aibasel/downward.git
cd downward
./build.py
```
Check whether `downward/fast-downward.py` exists.
To run solver, do
```bash
./<path-to-downwrad>/fast-downward.py path/to/pddl-domain.pddl path/to/problem.pddl --search "astar(blind())"
```

Example inside of `lego-workbenchmark`:
```bash
./downward/fast-downward.py src/pddl/coarse/lego_coarse.pddl src/problems_coarse/tier1/task_001.pddl --search "astar(blind())"
```


### Other
```bash
pip install pyyaml
```