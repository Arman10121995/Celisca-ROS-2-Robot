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

The containing published revision still needs its own complete CI result;
the isolated check does not replace the remote build and all test tiers.
Full failed Actions logs remain on the SSD; manifest hashes identify them.
