# Productive Multitasking for Industrial Robots

![alt text][digest]

[digest]: images/preemption.png "(top) Execution of a place skill in order to dispatch a piston rod on the tray of an AGV,
    while a disk is being detected on the conveyor belt.
    (bottom) Since this conveyor belt is running at a constant speed,
    waiting for the place skill to terminate would result in missing the opportunity to pick up the disk;
    the place skill is put on hold in order to start a pick skill directed towards the disk."

This repository lets you see and run an isolated version of the state machine/behavior tree code behind the paper "Productive Multitasking for Industrial Robots"
published at ICRA 2021:

```bibtex
@misc{wuthier2021productive,
      title={Productive Multitasking for Industrial Robots}, 
      author={D. Wuthier and F. Rovida and M. Fumagalli and V. Krüger},
      year={2021},
      eprint={2108.11471},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
    
## Installation (Windows, MacOS, Linux)

Install Python 3 with `numpy`.

## Usage

Simply execute `src/behavior_tree.py` or `src/finite_state_machine.py`.
The output shows how the behavior tree and the finite state machine evolve in two examples of simulated execution.
