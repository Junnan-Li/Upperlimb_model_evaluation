# Upperlimb_model_evaluation

## Installation Guide

> Notice: This repository is tested with Windows and OpenSim 4.5.

1. Install [OpenSim](https://simtk.org/projects/opensim) with [guidance](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53088790/Installing+OpenSim).
2. Configure the system to allow OpenSim [scripting with Matlab](https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53089380/Scripting+with+Matlab).
After this stage, you can verify your configuration by running this in Matlab:
   ```Matlab
   org.opensim.modeling.opensimCommon.GetVersion()
   ```
3. Clone the repository:
   ```bash
   git clone https://github.com/Junnan-Li/Upperlimb_model_evaluation.git
   ```
   > Warning: in OpenSim 4.5, there is an issue for visualization. We suggest you to clone the repo anywhere under in `C:\`
4.  (Optional) Switch to the branch that you need.
5. Initalize the submodule:
   ```bash
   git submodule update --init --recursive
   ```
6. Now load the Matlab project file by double clicking `Upper_limb.prj` in Matlab and it is fully loaded.

## Folder structure

- `examples`: Interactive example scripts for coding.
- `src`: Source code.
- `model`: OpenSim models used in project.
- `test`: Unit test classes.
- `resources`: Internal files generated from Matlab project.