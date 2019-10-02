# Autonomous Quad 2017

## To Run
Run ``RunSimSingle.m`` in MATLAB to run a single instance of the simulation. ``MonteCarlo.m`` is used to execute the simulation inside a Monte-Carlo run, adjust the script to change number of instances run.

## File Descriptions

### Main Directory
#### Scripts
| File | Description |
| - | - |
| ``RunSimSingle.m`` | Run single instance of simulation. |
| ``GetVerificationData.m`` |  |

#### Functions
| File | Description |
| - | - |
| ``RunSim.m`` |  |
| ``AverageProperties.m`` |  |
| ``GetProperties.m`` |  |
| ``LoadDecisions.m`` |  |
| ``GenerateExcel`` |  |

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
