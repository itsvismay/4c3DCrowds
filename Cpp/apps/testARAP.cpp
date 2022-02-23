#include <iostream>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include <igl/readOBJ.h>

int main(int, char**) {
	// Initialize Polyscope
	polyscope::init();
	polyscope::options::programName = "PBD Particles Test";
	
	// Read the mesh
	Eigen::MatrixXd meshV;
	Eigen::MatrixXi meshF;
	igl::readOBJ("../../../assets/donutsurface.obj", meshV, meshF);

	// Register the mesh with Polyscope
	polyscope::registerSurfaceMesh("input mesh", meshV, meshF);

	// Show the GUI
	polyscope::show();
}
