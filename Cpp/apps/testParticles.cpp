#include <iostream>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "hello.h"


// Parameters which we will set in the callback UI.
int nPts = 2000;
float anotherParam = 3.14;

void mySubroutine() {

  // do something useful...

  // Register a structure
  std::vector<glm::vec3> points;
  for (int i = 0; i < nPts; i++) {
    points.push_back(
        glm::vec3{ polyscope::randomUnit(), 
                   polyscope::randomUnit(), 
                   polyscope::randomUnit()
                  });
  }
  polyscope::registerPointCloud("my point cloud", points);
}

// Your callback functions
void myCallback() {

  // Since options::openImGuiWindowForUserCallback == true by default, 
  // we can immediately start using ImGui commands to build a UI

  ImGui::PushItemWidth(100); // Make ui elements 100 pixels wide,
                             // instead of full width. Must have 
                             // matching PopItemWidth() below.

  ImGui::InputInt("num points", &nPts);             // set a int variable
  ImGui::InputFloat("param value", &anotherParam);  // set a float variable

  if (ImGui::Button("run subroutine")) {
    // executes when button is pressed
    mySubroutine();
  }
  ImGui::SameLine();
  if (ImGui::Button("hi")) {
    polyscope::warning("hi");
  }

  ImGui::PopItemWidth();
}

int main(int, char**) {

    // Initialize polyscope, creating graphics contexts and constructing a window.
    // Should be called exactly once.
    polyscope::init();
    polyscope::options::programName = "PBD Particles Test";

    // Specify the callback
    polyscope::state::userCallback = myCallback;

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;

    return 0;
}