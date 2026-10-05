# CI extension dependency repair, October 5

Revision `9d97b8c` first failed during setup-ros on a transient ROS apt release
HTTP 404. A single rerun successfully installed ROS, built the workspace and
passed the fast tier, then failed the real native-state physics test because
latest Trimesh imports subscript NumPy dtype types unavailable in the Ubuntu
NumPy loaded by the sourced ROS Humble environment. The negative traceback is
retained in `trimesh-numpy-failure.txt`.

Both CI workflows now install Trimesh 4.1.8, compatible with that NumPy API.
An isolated SSD virtual environment using NumPy 1.21.6 and Trimesh 4.1.8 passes
the unchanged actual MuJoCo native-state/spawn test. The SciPy warning comes
from this host's unrelated newer inherited SciPy; CI supplies Ubuntu's SciPy.
The regular host's NumPy 2.2.6 / Trimesh 4.11.5 installation is preserved.

The exact published revision `36f38b5` passes [Actions run 37352290759](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/37352290759):
full build, 770 fast checks/four skips/one deselection, seven physics checks,
123 integration checks and registry validation. `ci-36f38b5-report.json` records
the actual job/step results. Later control/TurtleBot4 changes require their
own exact-revision CI result; this repair does not qualify robot missions.
Full failed Actions logs remain on the SSD; manifest hashes identify them.
