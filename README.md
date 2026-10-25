# MVA Robotics Exercises

My worked exercises for the **MVA (Mathématiques, Vision, Apprentissage) Robotics** course at Université Paris-Saclay — numerical robotics, differential kinematics, and simulation for robotics, built on [Pinocchio](https://github.com/stack-of-tasks/pinocchio).

> Course material originally from [TheotimeLH/2026_MVA_Robotics_Exercises](https://github.com/TheotimeLH/2026_MVA_Robotics_Exercises); this repo keeps my notebook solutions and learning notes.

## Highlights

- **Numerical robotics** (`0_introduction_to_numerical_robotics.ipynb` + `0_solutions.ipynb`) — rigid-body kinematics & dynamics with Pinocchio.
- **Differential IK** (`1_differential_IK.ipynb`) — Jacobian-based inverse kinematics on the Tiago robot (`tp1/tiago_loader.py`).
- **Simulation for robotics** (`2_simulation.ipynb`, `tp2/contact_dyn.py`) — contact dynamics, PGS vs ADMM/CCP contact solvers, differentiable simulation notes.
- **Motion planning (RRT)** (`3_RRT.ipynb`, `tp3/collision_wrapper.py`) — sampling-based planning on UR5: single-tree RRT (given), and my `BiRRT` (bidirectional / RRT-Connect) implementation with obstacle-avoidance collision checking.
- **Learning notes** (`docs/Robotics_Simulation_学习笔记.md`) — lecture notes covering collision detection (GJK), contact constraints (Signorini / Coulomb / Moreau), NCP solvers, ABA/RNEA, and differentiable simulation.

## Quick start

```bash
# 1. create conda env from provided yml
conda env create -f mva_robotics.yml
conda activate mva_robotics_2026

# 2. run notebooks
jupyter-lab
```

Meshcat visualization: open `localhost:700N/static/` where N is the Nth meshcat instance.

## Layout

```
├── 0_introduction_to_numerical_robotics.ipynb   # base notebook
├── 0_solutions.ipynb                            # my solutions
├── 1_differential_IK.ipynb                      # differential IK on Tiago
├── 2_simulation.ipynb                           # contact simulation
├── 2_simulation_with_solutions.ipynb            # teacher's TP2 solutions
├── 3_RTT.ipynb                                  # motion planning: RRT / BiRRT (my impl)
├── tp1/ tiago_loader.py
├── tp2/ contact_dyn.py, visualization.py
├── tp3/ collision_wrapper.py
├── utils/ meshcat viewer wrapper, UR5 loaders
├── docs/ Robotics_Simulation_学习笔记.md
├── mva_robotics.yml                             # conda env
└── pinocchio_cheat_sheet.pdf
```

## License

See `LICENSE` (original course repo).
