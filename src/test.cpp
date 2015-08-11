#include <iostream>
#include "base/matrix.hpp"
#include "base/data.hpp"
#include "core/registrator.hpp"
#include "core/cpd_rigid.hpp"
#include "core/cpd_nonrigid.hpp"

using namespace cpd;

using namespace Eigen;
using namespace std;

int main()
{
    MatrixType<float, 3>::MatrixD model, data;
    //getInputData<float, 3>("face2.txt", "face1.txt", model, data);
   // getInputData<float, 3>("points-test.txt", "points.txt", model, data);
    getInputData<float, 3>("lily-5.txt", "lily-30.txt", model, data);
    CPDNRigid<float, 3>* reg = new CPDNRigid<float, 3>();
    //CPDRigid<float, 3>* reg = new CPDRigid<float, 3>();
    reg->setInputData(model, data);
    reg->setVision(true);
    reg->setIterativeNumber(100);
    reg->setVarianceTolerance(1e-6);
    reg->setEnergyTolerance(1e-5);
    reg->setOutlierWeight(0.0);
    reg->setFgtFlag(true);
    reg->setFgtEpsilon(1e-4);
  //  reg->setLowRankFlag(true);
  //  reg->setKLowRank(40);
    reg->run();

    std::cout << reg->getModel() << std::endl;

}
