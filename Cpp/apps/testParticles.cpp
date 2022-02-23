#include <iostream>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "hello.h"
#include <Eigen/Core>


// Parameters which we will set in the callback UI.
float anotherParam = 3.14;

struct ParticleSystem
{
  std::vector<glm::vec3> pos;
  std::vector<glm::vec3> vel;
  std::vector<float> mass;

};

void simulate() {


}

void render() {

}

// Your callback functions
void update() {
  simulate();
  render();
  
}

void initScene1(ParticleSystem& ps, int len, int wid, int hgt, float radius){
    // Register a structure
    for(int iy = 0; iy<(hgt/2/radius); ++iy){
      for(int ix =0; ix<(len/2/radius); ++ix){
        for(int iz=0; iz<(wid/2/radius); ++iz){

          float x = ix*radius;
          float y = iy*radius;
          float z = iz*radius;
          ps.pos.emplace_back(x, y, z);
          ps.vel.emplace_back(x, y, z);
          std::cout<<x <<", "<<y<<", "<<z<<std::endl;
        }
      }
    }
}

int main(int, char**) {

    // Initialize polyscope, creating graphics contexts and constructing a window.
    // Should be called exactly once.
    polyscope::init();
    polyscope::options::programName = "PBD Particles Test";

    //Initialize Scene
    ParticleSystem ps;
    float rad = 0.2;
    initScene1(ps, 2, 2, 5, rad);

    // visualize!
    polyscope::PointCloud* psCloud = polyscope::registerPointCloud("points", ps.pos);
    // set some options
    psCloud->setPointRadius(rad);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);

    // Specify the callback
    polyscope::state::userCallback = update;

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;

    return 0;
}