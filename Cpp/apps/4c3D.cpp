#include <iostream>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

#include <Eigen/Core>

struct Scene{
 Eigen::MatrixXd q; // rows = all nodes for all agents, columns = x,y,z,t
 std::vector<std::array<int, 2>> edges;
 Eigen::VectorXi Agents;
};


void mySubroutine(int axes, Scene& scene, Eigen::MatrixXd& nodes, polyscope::CurveNetwork* pcn) {
  // change the render frame
  switch (axes)
  {
  case 012:
    nodes.col(0) = scene.q.col(0);
    nodes.col(1) = scene.q.col(1);
    nodes.col(2) = scene.q.col(2);
    pcn->updateNodePositions(nodes);
    break;
  case 013:
    nodes.col(0) = scene.q.col(0);
    nodes.col(1) = scene.q.col(1);
    nodes.col(2) = scene.q.col(3);
    pcn->updateNodePositions(nodes);
    break;
  case 023:
    nodes.col(0) = scene.q.col(0);
    nodes.col(1) = scene.q.col(2);
    nodes.col(2) = scene.q.col(3);
    pcn->updateNodePositions(nodes);
    break;
  case 123:
    nodes.col(0) = scene.q.col(1);
    nodes.col(1) = scene.q.col(2);
    nodes.col(2) = scene.q.col(3);
    pcn->updateNodePositions(nodes);
    break;
  default:
    break;
  }
  
}

//Setup Test Curves
void setupTestCurves(Scene& scene){

  double revolutions = 0.5;
  int numNodes = 40;
  int numAgents = 1;
  scene.q.resize(numNodes*numAgents, 4);
  scene.Agents.resize(numNodes*numAgents);
  
  //For each agent
  //For num nodes, create from t=0..end inclusive, a set of points
  for(int a=1; a<numAgents+1; a++){
    for(int i=0; i<numNodes; i++){
      double thetai = i*(2*3.14159*revolutions - 0)/(numNodes-1);
      scene.q(a*i, 0) = cos(thetai);
      scene.q(a*i, 1) = sin(thetai);
      scene.q(a*i, 2) = 0;  
      scene.q(a*i, 3) = i*(10.0 - 0)/(numNodes-1);
      scene.Agents(a*i) = a;
      if(i<numNodes-1)
        scene.edges.push_back({i,i+1});
    }
  }  

}

int main(int, char**) {

    // Initialize polyscope, creating graphics contexts and constructing a window.
    // Should be called exactly once.
    polyscope::init();
    polyscope::options::programName = "PBD Particles Test";


    // Create a structure
    Scene scene;
    setupTestCurves(scene);
    Eigen::MatrixXd nodes = Eigen::MatrixXd::Zero(scene.q.rows(),3);
    nodes.col(0) = scene.q.col(0);
    nodes.col(1) = scene.q.col(1);
    nodes.col(2) = scene.q.col(2);
    
    // Register polyscope structure
    // Add the curve network
    polyscope::CurveNetwork* pcn = polyscope::registerCurveNetwork("my network", nodes, scene.edges);

    // Specify the callback
    polyscope::state::userCallback = [&](){
      // Since options::openImGuiWindowForUserCallback == true by default, 
      // we can immediately start using ImGui commands to build a UI
      ImGui::PushItemWidth(100); // Make ui elements 100 pixels wide,
      if (ImGui::Button("XYZ")) {
        // executes when button is pressed
        mySubroutine(012, scene, nodes, pcn);
      }
      if (ImGui::Button("XYT")) {
        // executes when button is pressed
        mySubroutine(013, scene, nodes, pcn);
      }
      if (ImGui::Button("YZT")) {
        // executes when button is pressed
        mySubroutine(123, scene, nodes, pcn);
      }
      if (ImGui::Button("XZT")) {
        // executes when button is pressed
        mySubroutine(023, scene, nodes, pcn);
      }
    };

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;

    return 0;
}