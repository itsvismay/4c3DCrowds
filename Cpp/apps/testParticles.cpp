#include <iostream>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include <Eigen/Core>
#include "SpatialHash.h"

float h = 1.0f/60.0f;
Eigen::Vector3f grav(0, -10.0f,0);
float rad = 0.1f;
int HASH_SIZE = 100;
float totalTime = 0;

//From optimized spatial hashing paper
float cellSize = 500*rad;
float invGridSpacing = 1.0/(4.5*rad);
std::map<std::tuple<int, int, int>, std::vector<int>> SpatialHash;

struct ParticleSystem
{
  int numParticles = 0;
  //watch out for stl/eigen bugs. Vector3f should be fine accordint to docs
  std::vector<Eigen::Vector3f> guesspos;
  std::vector<Eigen::Vector3f> pos;
  std::vector<Eigen::Vector3f> vel;
  std::vector<float> mass;
  std::vector<float> rad;

};

void simulate(ParticleSystem& ps, Hash& spH) {
  //spH.findNeighbors(ps.pos);
  findNeighbors(ps.pos);

  // do 10 substeps
  float dt = h/1.0f;
  for(int nsteps = 0; nsteps<1; nsteps++){   // predict
    for(int i=0; i<ps.numParticles; i++){
      ps.guesspos[i] = ps.pos[i] + dt*ps.vel[i] + dt*dt*(1.0/ps.mass[i])*grav;
    }

    // //--------------Check hash---------------------------
    //  //constraints
    // for(int i=0; i<ps.numParticles; i++){
    //   Eigen::Vector3f pi = ps.guesspos[i];
    //   std::tuple<int, int, int> xyzcells = xyzhash(pi); 
    //   std::map<std::tuple<int, int, int>, std::vector<int>>::iterator it = SpatialHash.find(xyzcells);
    //   if(it == SpatialHash.end()){  
    //     continue;
    //   }
    //   int numNeighbors = it->second.size(); 
    //   std::cout<<"Particle: "<<i<<std::endl;
    //   std::cout<<"  neighbors: "<<it->second.size()<<std::endl;
    //   for(int jj=0; jj<it->second.size(); jj++)
    //   {
    //     int j = it->second[jj];

    //     if(i==j){
    //       continue;
    //     }
    //     Eigen::Vector3f pj = ps.guesspos[j];

    //     Eigen::Vector3f nij = pi - pj;
    //     std::cout<<"  Dists: "<<nij.norm()<<",";
    //   }
    //   std::cout<<std::endl;

    //   std::cout<<"  Real Dists: ";
    //   std::vector<float> dists;
    //   for(int j=0; j<ps.numParticles; j++){
    //     Eigen::Vector3f pj = ps.guesspos[j];
    //     Eigen::Vector3f nij = pi - pj;
    //     dists.push_back(nij.norm());
    //   }
    //   std::cout<<std::endl;
    //   std::cout<<"Sorted Dists"<<dists.size()<<": "<<std::endl;
    //   std::sort(dists.begin(), dists.end());
    //   for(int i=0; i<dists.size(); i++)
    //   {
    //     std::cout<<dists[i]<<", ";
    //   }
    // }
    // //-----------------------------------------

    //constraints
    for(int i=0; i<ps.numParticles; i++){
      Eigen::Vector3f pi = ps.guesspos[i];
      // //Neighbor constraints for sand
      // int first = spH.firstNeighbor[i];
			// int numNeighbors = spH.firstNeighbor[i + 1] - first;

      // for(int jj=0; jj<numNeighbors; jj++)
      // {
      //   int j = spH.neighbors[first + jj];
      
      std::tuple<int, int, int> xyzcells = xyzhash(pi); 
      std::map<std::tuple<int, int, int>, std::vector<int>>::iterator it = SpatialHash.find(xyzcells);
      if(it == SpatialHash.end()){  
        continue;
      }
      int numNeighbors = it->second.size(); 
      for(int jj=0; jj<it->second.size(); jj++)
      {
        int j = it->second[jj];

        if(i==j){
          continue;
        }
        Eigen::Vector3f pj = ps.guesspos[j];

        Eigen::Vector3f nij = pi - pj;
        float r = nij.norm(); //dist between particles
        if(r>0){
          nij = nij/r;
        }
        if(r < 4*rad){
          float pushDist = 0.5f*(4*rad - r);
          ps.guesspos[j] -= nij*pushDist;
          ps.guesspos[i] += nij*pushDist;
        }

      }

        //particle bounds in -y direction
        if (ps.guesspos[i][1]<0){
          ps.guesspos[i][1] = 0;
        }
        //particle bounds in -z direction
        if (ps.guesspos[i][2]<-2){
          ps.guesspos[i][2] = -2;
        }
        //particle bounds in +z direction
        if (ps.guesspos[i][2]>2){
          ps.guesspos[i][2] = 2;
        }
        //particle bounds in -x direction
        if (ps.guesspos[i][0]<-2){
          ps.guesspos[i][0] = -2;
        }
        //particle bounds in +x direction
        if (ps.guesspos[i][0]>2.5+sin(totalTime)){
          ps.guesspos[i][0] = 2.5+sin(totalTime);
        }
    }

    //update vel
    float maxdp = 0.4*rad;
    for(int i=0; i<ps.numParticles; i++){
      Eigen::Vector3f dp = (ps.guesspos[i] - ps.pos[i]);
      if(dp.norm()>maxdp){
          dp *= (maxdp/dp.norm()); //normalize to maxdp
      }
      ps.vel[i] = dp/dt;
      ps.pos[i] += dp;
    }
  }
  totalTime += h;

}

void render(ParticleSystem& ps, polyscope::PointCloud* psCloud) {
  psCloud->updatePointPositions(ps.pos);
}


void initScene1(ParticleSystem& ps, int len, int wid, int hgt, float radius){
    // Register a structure
    float d = 2*radius;
    for(int iy = 0; iy<(hgt/d); ++iy){
      for(int ix =0; ix<(len/d); ++ix){
        for(int iz=0; iz<(wid/d); ++iz){
          Eigen::Vector3f r((float) rand() / RAND_MAX, (float) rand() / RAND_MAX, (float) rand() / RAND_MAX);
          r = r*(radius/100);
          float x = ix*d+radius + r[0];
          float y = iy*d+radius + r[1];
          float z = iz*d+radius + r[2];
          ps.pos.emplace_back(x, y, z);
          ps.guesspos.emplace_back(x, y, z);
          ps.vel.emplace_back(0, 0, 0);
          ps.mass.emplace_back(1.0f);
          ps.rad.emplace_back(radius);
          ps.numParticles += 1;
        }
      }
    }
}

int main(int, char**) {
    // Initialize polyscope, creating graphics contexts and constructing a window.
    // Should be called exactly once.
    polyscope::init();
    polyscope::options::programName = "PBD Particles Test";
    polyscope::options::groundPlaneMode  = polyscope::GroundPlaneMode::None;
    polyscope::view::upDir = polyscope::UpDir::YUp; 
    //Initialize Scene
    ParticleSystem ps;
    initScene1(ps, 2, 2.5, 2, rad);
    Hash spHash(ps.numParticles);

    // visualize!
    polyscope::PointCloud* psCloud = polyscope::registerPointCloud("points", ps.pos);
  
    // set some options
    psCloud->setPointRadius(rad);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Sphere);

    // Specify the callback
    polyscope::state::userCallback = [&](){
      simulate(ps,spHash);
      render(ps, psCloud);

    };

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;

    return 0;
}