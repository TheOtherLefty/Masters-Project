# Autonomous Quad 2019

## To Run
Run ``RunSimSingle.m`` in MATLAB to run a single instance of the simulation. ``MonteCarlo.m`` is used to execute the simulation inside a Monte-Carlo run, adjust the script to change number of instances run.

## Branch Descriptions

| Branch | Description |
| Master | Contains original simulation. UAV searches 5x5 grid for 3 objects (prism, cube, sphere), deposit base is located at coordinate (0,0). |
| Symmetry | Updated implementation, base coordinate moved to center of a 10x10 grid. Symmetrically applies the same controller 4 times for each 5x5 quadrant, transitioning between the two. |

### Main Directory
#### Scripts
| File | Description |
| - | - |
| ``RunSimSingle.m`` | Run single instance of simulation. |
| ``MonteCarlo.m`` | Run multiple instances of simulation and output data to excel spreadsheet "MCResults.xslx" |

#### Classes
| File | Description |
| - | - |
| ``cAgent.m`` | Dynamic agent superclass. |
| ``cTarget.m`` | Target class with properties and methods. Subclass of ``cAgent``. |
| ``cEnvironment.m`` | Environment class describes properties and methods of UAV environment. |
| ``cBlackbox`` | Blackbox class contains methods for reading data from simulation and then presenting it as animations or plots. |

### @cQuadrotor
| File | Description |
| - | - |
| ``cQuadrotor.m`` | Quadrotor class with properties and class constructor. Subclass of ``cAgent``. |
| ``SmartSearch.m`` | Controller for PRISM files, implements an optimal search pattern using PRISM generated state and transition files. |
| ``SearchPattern.m`` | Implements a pattern search used to compare performance against PRISM search. 5x5 pattern search is a square wave; 10x10 pattern search is a spiral inwards towards the base. |

