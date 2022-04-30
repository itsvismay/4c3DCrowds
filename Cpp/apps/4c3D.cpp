#include <iostream>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "spring.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>

struct Scene{
  int numNodes = 40;
  int numAgents = 1;
  Eigen::MatrixXd qInit; // rows = all nodes for all agents, columns = x,y,z,t
  std::vector<std::array<int, 2>> edges;
  Eigen::VectorXi Agents;
  int axes = 012;
  std::vector<int> pickOut;
  Eigen::SparseMatrix<double> P;

  Eigen::VectorXd x0,x,xdot;
  Eigen::SparseMatrix<double> M, K;
  double springK = 10;
};


void pickOutMatrix(std::vector<int>& pickOut, int size, Eigen::SparseMatrix<double>& P){
  P.resize(size - 4*pickOut.size(), size);
  // std::vector<Eigen::Triplet<int>> pTrips;
  std::sort (pickOut.begin(), pickOut.end());
  int j=0, pInd = 0;

  for(int i=0; i<size/4; i++){
    if(i==pickOut[pInd]){
      pInd++;
      continue;
    }
    P.coeffRef(4*j + 0, 4*i + 0) = 1;
    P.coeffRef(4*j + 1, 4*i + 1) = 1;
    P.coeffRef(4*j + 2, 4*i + 2) = 1;
    P.coeffRef(4*j + 3, 4*i + 3) = 1;
    j++;
  }
}

void mySubroutine(int axes, Scene& scene, Eigen::MatrixXd& nodes, polyscope::CurveNetwork* pcn) { 
  
  //change the render frame
  switch (axes)
  {
  case 012:
    nodes.col(0) = scene.qInit.col(0);
    nodes.col(1) = scene.qInit.col(1);
    nodes.col(2) = scene.qInit.col(2);
    pcn->updateNodePositions(nodes);
    break;
  case 013:
    nodes.col(0) = scene.qInit.col(0);
    nodes.col(1) = scene.qInit.col(1);
    nodes.col(2) = scene.qInit.col(3);
    pcn->updateNodePositions(nodes);
    break;
  case 023:
    nodes.col(0) = scene.qInit.col(0);
    nodes.col(1) = scene.qInit.col(2);
    nodes.col(2) = scene.qInit.col(3);
    pcn->updateNodePositions(nodes);
    break;
  case 123:
    nodes.col(0) = scene.qInit.col(1);
    nodes.col(1) = scene.qInit.col(2);
    nodes.col(2) = scene.qInit.col(3);
    pcn->updateNodePositions(nodes);
    break;
  default:
    break;
  }
}

void initPhysicsParameters(Scene& scene){
  // Setup positions
  scene.x0 = scene.qInit.reshaped<Eigen::RowMajor>();
  scene.x = scene.qInit.reshaped<Eigen::RowMajor>();

  // Setup Mass -----------------------------
  scene.M.resize(4*scene.numNodes*scene.numAgents, 4*scene.numNodes*scene.numAgents);
  scene.M.setIdentity();

  // Setup K
  scene.K.resize(scene.x.size(), scene.x.size());
  spring::hessian(scene.x, scene.edges, scene.springK, scene.K);


  // Setup  Velocities ----------------------
  scene.xdot = Eigen::VectorXd::Zero(scene.x.size());
}

void setupSceneOneTest(Scene& scene){
  scene.numAgents = 1;
  scene.numNodes = 40;

  // Setup Curve -----------------------------
  double revolutions = 0.5;
  int endtime = 2.0;
  scene.qInit.resize(scene.numNodes*scene.numAgents, 4);
  scene.Agents.resize(scene.numNodes*scene.numAgents);
  
  //For each agent
  //For num nodes, create from t=0..end inclusive, a set of points
  for(int a=1; a<scene.numAgents+1; a++){
    for(int i=0; i<scene.numNodes; i++){
      double thetai = i*(2*3.14159*revolutions - 0)/(scene.numNodes-1);
      scene.qInit(a*i, 0) = cos(thetai);
      scene.qInit(a*i, 1) = sin(thetai);
      scene.qInit(a*i, 2) = 0;  
      scene.qInit(a*i, 3) = i*(endtime - 0)/(scene.numNodes-1);
      scene.Agents(a*i) = a;
      if(i<scene.numNodes-1)
        scene.edges.push_back({a*i,a*i+1});
    }
    scene.pickOut.push_back((a-1)*scene.numNodes+0);
    scene.pickOut.push_back((a)*scene.numNodes-1);
  } 
}

void setupSceneTwoTest(Scene& scene){
  scene.numAgents = 2;
  scene.numNodes = 40;

  // Setup Curve -----------------------------
  double revolutions = 0.5;
  int endtime = 2.0;
  scene.qInit.resize(scene.numNodes*scene.numAgents, 4);
  scene.qInit.setZero();
  scene.Agents.resize(scene.numNodes*scene.numAgents);
  
  //For each agent
  //For num nodes, create from t=0..end inclusive, a set of points
  for(int a=1; a<scene.numAgents+1; a++){
    for(int i=0; i<scene.numNodes; i++){

      if(a == 1){
        double thetai = i*(2*3.14159*revolutions - 0)/(scene.numNodes-1);
        scene.qInit(scene.numNodes*(a-1)+i, 0) = cos(thetai);
        scene.qInit(scene.numNodes*(a-1)+i, 1) = sin(thetai);
        scene.qInit(scene.numNodes*(a-1)+i, 2) = 0;  
        scene.qInit(scene.numNodes*(a-1)+i, 3) = i*(endtime - 0.0)/(scene.numNodes-1.0);
        scene.Agents(scene.numNodes*(a-1)+i) = a;

      }else{
        double thetai = i*(2*3.14159*revolutions - 0)/(scene.numNodes-1);
        scene.qInit(scene.numNodes*(a-1)+i, 0) = 0;
        scene.qInit(scene.numNodes*(a-1)+i, 1) = -sin(thetai);
        scene.qInit(scene.numNodes*(a-1)+i, 2) = cos(thetai);;  
        scene.qInit(scene.numNodes*(a-1)+i, 3) = i*(endtime - 0.0)/(scene.numNodes-1.0);
        scene.Agents(scene.numNodes*(a-1)+i) = a; 
      }
      if(i<scene.numNodes-1){
        scene.edges.push_back({scene.numNodes*(a-1)+i,scene.numNodes*(a-1)+i+1});
        std::cout<<scene.numNodes*(a-1)+i<<", "<<scene.numNodes*(a-1)+i+1<<std::endl;
      }
    }
    scene.pickOut.push_back((a-1)*scene.numNodes+0);
    scene.pickOut.push_back((a)*scene.numNodes-1);
  }
}

//Setup Test Curves
void setupTestCurves(Scene& scene){
  //Setup Scene 1
  //setupSceneOneTest(scene);
  setupSceneTwoTest(scene);




  //Scene physics setup
  initPhysicsParameters(scene);
  pickOutMatrix(scene.pickOut, scene.x.size(), scene.P);
}

void fd_test_gradient(Scene& scene){
  double eps = 1e-5;
  double E0 = spring::energy(scene.x, scene.edges, scene.springK);
  Eigen::VectorXd fdGrad = Eigen::VectorXd::Zero(scene.x.size());
  for(int i=0; i<scene.x.size(); i++){
    scene.x(i) += 0.5*eps;
    double E1 = spring::energy(scene.x, scene.edges, scene.springK);
    scene.x(i) -= 0.5*eps;

    scene.x(i) -= 0.5*eps;
    double E2 = spring::energy(scene.x, scene.edges, scene.springK);
    scene.x(i) += 0.5*eps;

    fdGrad(i) = (E1 - E2)/eps;
  }

  Eigen::VectorXd grad = Eigen::VectorXd::Zero(scene.x.size());
  spring::gradient(scene.x, scene.edges, scene.springK, grad);
  std::cout<<grad.transpose()<<std::endl<<std::endl;
  std::cout<<fdGrad.transpose()<<std::endl;
  std::cout<<(fdGrad-grad).norm()<<std::endl<<std::endl;
}

void fd_test_hessian(Scene& scene){
  double eps = 1e-5;
  Eigen::VectorXd G0 = Eigen::VectorXd::Zero(scene.x.size());
  spring::gradient(scene.x, scene.edges, scene.springK, G0);

  Eigen::MatrixXd fdH; fdH.resize(scene.x.size(), scene.x.size()); fdH.setZero();

  Eigen::VectorXd G1 = Eigen::VectorXd::Zero(scene.x.size());
  Eigen::VectorXd G2 = Eigen::VectorXd::Zero(scene.x.size());
  for(int i=0; i<scene.x.size(); i++){
    G1.setZero();
    G2.setZero();

    scene.x(i) += 0.5*eps;
    spring::gradient(scene.x, scene.edges, scene.springK, G1);
    scene.x(i) -= 0.5*eps;

    scene.x(i) -= 0.5*eps;
    spring::gradient(scene.x, scene.edges, scene.springK, G2);
    scene.x(i) += 0.5*eps;

    fdH.row(i) = (G1 - G2)/eps;
  }

  Eigen::SparseMatrix<double> realH; realH.resize(scene.x.size(), scene.x.size());
  spring::hessian(scene.x, scene.edges, scene.springK, realH);
  Eigen::MatrixXd H = realH;
  std::cout<<H.block<16,16>(0,0)<<std::endl<<std::endl;
  std::cout<<fdH.block<16,16>(0,0)<<std::endl<<std::endl;
}

void newtons_method(Scene& scene){
  int MAXIT = 100;
  Eigen::VectorXd x_k = scene.x;
  double h = 1e-1;
  Eigen::VectorXd f_k = Eigen::VectorXd::Zero(scene.x.size());
  std::cout<<scene.P.rows()<<std::endl;
  Eigen::SparseMatrix<double> PIPt(scene.P.rows(), scene.P.rows()); PIPt.setIdentity();

  Eigen::VectorXd Px_0 = scene.P*scene.x;
  Eigen::VectorXd Px_k = scene.P*scene.x;
  Eigen::VectorXd Pf_k = scene.P*f_k;
  Eigen::VectorXd Pg_k = Pf_k;
  Eigen::VectorXd Px_dot = scene.P*scene.xdot;
  Eigen::SparseMatrix<double> PiMPt = scene.P*scene.M*scene.P.transpose();
  spring::hessian(x_k, scene.edges, scene.springK, scene.K);
  Eigen::SparseMatrix<double> PKPt = scene.P*scene.K*scene.P.transpose();
  
  Eigen::SparseMatrix<double> Gg_k = PIPt - h*h*PiMPt*PKPt;
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> sqr;
  sqr.compute(Gg_k);

  Eigen::VectorXd Pdx = Eigen::VectorXd::Zero(scene.P.rows());

  for (int k = 0; k<MAXIT; k++){
    spring::gradient(x_k, scene.edges, scene.springK, f_k);
    f_k*=-1; //force is - grad

    Pf_k = scene.P*f_k;
    Px_k = scene.P*x_k;

    Pg_k = Px_k - (Px_0 + h*(Px_dot + h*PiMPt*Pf_k));

    Pdx = -sqr.solve(Pg_k);

    x_k += scene.P.transpose()*0.5*Pdx;

    if(x_k != x_k)
    {
      std::cout<<"NANs!!"<<std::endl;
      std::cout<<k<<std::endl;
      break;
    }

    if(Pg_k.squaredNorm()< 1e-4)
    {
      std::cout<<"converged"<<std::endl;
      //std::cout<<g_k.squaredNorm()<<std::endl;
      break;
    }

    if(k>90)
    {
      std::cout<<"NM not converging."<<std::endl;
      break;
    }

  }

  scene.xdot = (x_k - scene.x)/h;
  scene.x = x_k;

  //x to qInit
  //For each agent
  //For num nodes, create from t=0..end inclusive, a set of points
  int xind = 0;
  for(int a=1; a<scene.numAgents+1; a++){
    for(int i=0; i<scene.numNodes; i++){
      scene.qInit(scene.numNodes*(a-1)+i, 0) = scene.x(xind); xind++;
      scene.qInit(scene.numNodes*(a-1)+i, 1) = scene.x(xind); xind++;
      scene.qInit(scene.numNodes*(a-1)+i, 2) = scene.x(xind); xind++;  
      scene.qInit(scene.numNodes*(a-1)+i, 3) = scene.x(xind); xind++;
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
    Eigen::MatrixXd nodes = Eigen::MatrixXd::Zero(scene.qInit.rows(),3);
    nodes.col(0) = scene.qInit.col(0);
    nodes.col(1) = scene.qInit.col(1);
    nodes.col(2) = scene.qInit.col(2);
    
    // // Register polyscope structure
    // // Add the curve network
    polyscope::CurveNetwork* pcn = polyscope::registerCurveNetwork("my network", nodes, scene.edges);
  

    // Specify the callback
    polyscope::state::userCallback = [&](){
      // Since options::openImGuiWindowForUserCallback == true by default, 
      // we can immediately start using ImGui commands to build a UI
      ImGui::PushItemWidth(100); // Make ui elements 100 pixels wide,
      if (ImGui::Button("XYZ")) {
        // executes when button is pressed
        scene.axes = 012;
      }
      if (ImGui::Button("XYT")) {
        // executes when button is pressed
        scene.axes = 013;
      }
      if (ImGui::Button("YZT")) {
        // executes when button is pressed
        scene.axes = 123;
      }
      if (ImGui::Button("XZT")) {
        // executes when button is pressed
        scene.axes = 023;
      }

      mySubroutine(scene.axes, scene, nodes, pcn);
      newtons_method(scene);
      double e = spring::energy(scene.x, scene.edges, scene.springK);
      std::cout<<e<<std::endl;
    };

    // Give control to the polyscope gui
    polyscope::show();


    return EXIT_SUCCESS;

    return 0;
}



