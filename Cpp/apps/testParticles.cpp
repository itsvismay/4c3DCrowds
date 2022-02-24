#include <iostream>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include <Eigen/Core>

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

//64 bit return value should help
inline size_t hash(Eigen::Vector3f &p, float invGridSpacing, const int n) {

    int ix = (unsigned int)((p[0]) *invGridSpacing);
    int iy = (unsigned int)((p[1]) *invGridSpacing);
    int iz = (unsigned int)((p[2]) *invGridSpacing);
    return (unsigned int)((ix * 92837111) ^ (iy * 689287499) ^ (iz * 83492791)) % n;
}

//64 bit return value should help
std::tuple<int, int, int> xyzhash(Eigen::Vector3f &p) {

     float px = p[0];
      //get xcell
      int xcell = floor((px - -10)/cellSize);
      float py = p[1];
      //get ycell
      int ycell = floor((py - -1)/cellSize);
      float pz = p[2];
      //get zcell
      int zcell = floor((pz - -10)/cellSize);

      std::tuple<int, int, int> xyzcells{xcell, ycell, zcell};
      return xyzcells;
}

void findNeighbors(std::vector<Eigen::Vector3f>& pos){
  SpatialHash.clear();
  //HASH dimensions
  //-10 to 10 on x
  // -1 to 20 on y
  // -10 to 10 on z

  for(int i=0; i<pos.size(); i++){
      std::tuple<int, int, int> xyzcells = xyzhash(pos[i]);
      std::map<std::tuple<int, int, int>, std::vector<int>>::iterator it = SpatialHash.find(xyzcells);
      if(it == SpatialHash.end()){
        std::vector<int> valu; valu.push_back(i);
        SpatialHash.insert(std::make_pair(xyzcells, valu));
      }else{
        it->second.push_back(i);
      }
  }
}

struct Hash{
  int size=307110;

  int* first = new int[307110];
  int* marks = new int[307110];
  int currentMark = 0;

  int* next;
  int* firstNeighbor;
	std::vector<int> neighbors;
  float origx = -100;
  float origy = -1;
  float origz = -100;
  Hash(int maxParticles){
    this->next = new int[maxParticles];
    this->firstNeighbor = new int[maxParticles + 1];
    this->neighbors.resize(10 * maxParticles);

  }; 
  void findNeighbors(std::vector<Eigen::Vector3f>& pos){
    this->currentMark++;
		for (int i = 0; i < pos.size(); i++) {
			float px = pos[i][0];
			float py = pos[i][1];
      float pz = pos[i][2];
			
			int gx = floor((px - origx) * invGridSpacing);
			int gy = floor((py - origy) * invGridSpacing);
			int gz = floor((pz - origz) * invGridSpacing);
			
			int h = (abs((gx * 92837111) ^ (gy * 689287499) ^ (gz * 83492791))) % this->size;
						
			if (this->marks[h] != this->currentMark) {				
				this->marks[h] = this->currentMark;
				this->first[h] = -1;
			}

			this->next[i] = this->first[h];
			this->first[h] = i;
		}
		
		// collect neighbors
		
		this->neighbors.clear();

		float h2 = (1.0/invGridSpacing)*(1.0/invGridSpacing);

		for (int i = 0; i < pos.size(); i++) {
			this->firstNeighbor[i] = this->neighbors.size();
			float px = pos[i][0];
			float py = pos[i][1];
      float pz = pos[i][2];
			
			int gx = floor((px - origx) * invGridSpacing);
			int gy = floor((py - origy) * invGridSpacing);
			int gz = floor((pz - origz) * invGridSpacing);
						
			int x, y;
			
			for (x = gx - 1; x <= gx + 1; x++) {
				for (y = gy - 1; y <= gy + 1; y++) {
						
					int h = (abs((gx * 92837111) ^ (gy * 689287499) ^ (gz * 83492791))) % this->size;

					if (this->marks[h] != this->currentMark) 
						continue;
				
					int id = this->first[h];
					while (id >= 0) 
					{
						float dx = pos[id][0] - px;
						float dy = pos[id][1] - py;
            float dz = pos[id][2] - py;
						
						if ((dx * dx + dy * dy + dz*dz )< h2) 
							neighbors.push_back(id);

						id = this->next[id];						
					}
				}
			}
		}
		firstNeighbor[pos.size()] = neighbors.size();
  }

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