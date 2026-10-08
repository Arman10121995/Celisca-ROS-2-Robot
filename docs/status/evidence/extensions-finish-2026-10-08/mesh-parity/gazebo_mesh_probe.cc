#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <iostream>
#include <iomanip>
int main(int argc, char **argv) {
  std::cout << std::setprecision(15);
  for (int i=1; i<argc; ++i) {
    const auto *m = ignition::common::MeshManager::Instance()->Load(argv[i]);
    if (!m) return 2;
    auto low=m->Min(), high=m->Max();
    std::cout << "{\"mesh\":\"" << argv[i] << "\",\"vertices\":" << m->VertexCount()
      << ",\"lower\":[" << low.X() << ',' << low.Y() << ',' << low.Z()
      << "],\"upper\":[" << high.X() << ',' << high.Y() << ',' << high.Z() << "]}\n";
  }
}
