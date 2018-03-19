#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Core>

#include <sophus/se3.h>
#include <sophus/so3.h>

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using Sophus::SE3;
using Sophus::SO3;

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "Usage: pose_graph_gtsam shpere.g2o" << endl;
        return 1;
    }

    ifstream fin(argc[1]);
    if (!fin)
    {
        cout << "file " << argv[1] << " does not exist\n";
        return 1;
    }
    
    gtsam::NonlinearFactorGraph::shared_ptr graph(new gtsam::NonlinearFactorGraph);

    gtsam::Values::shared_ptr initial(new gtsam::Values);

    int cntVertex = 0, cntEdge = 0;
    cout << "reading from g2o file\n";

    while (!fin.eof()){
        string tag;
        fin >> tag;
        if (tag == "VERTEXSE3:QUAT")
        {
            gtsam::Key id;
            fin >> id;
            double data[7];
            for (int i = 0; i < 7; i++)
                fin >> data[i];

            gtsam::Rot3 R = gtsam::Rot3::Quaternion(data[6], data[3], data[4], data[5]);
            gtsam::Point3 t(data[0], data[1], data[2]);
            initial->insert(id, gtsam::Pose3(R, t));
            cntVertex++;
        }
        else if (tag == "EDGE_SE3:QUAT")
        {
            gtsam::Matrix m = gtsam::I_6x6;
            gtsam::Key id1, id2;
            fin >> id1 >> id2;

            double data[7];
            for (int i = 0; i < 7; i++)
                fin >> data[i];

            gtsam::Rot3 R = gtsam::Rot3::Quaternion(data[6], data[3], data[4], data[5]);
            gtsam::Point3 t(data[0], data[1], data[2]);
            
            for (int i = 0; i < 6; i++)
                for (int j = i; j < 6; j++)
                {
                    double mij;
                    fin >> mij;

                    m(i, j) = mij;
                    m(j, i) = mij;
                }

            gtsam::Matrix mgtsam = gtsam::I_6x6;
            mgtsam.block<3, 3>(0, 0) = m.block<3, 3>(3, 3);
            mgtsam.block<3, 3>(3, 3) = m.block<3, 3>(0, 0);
            mgtsam.block<3, 3>(0, 3) = m.block<3, 3>(0, 3);
            mgtsam.block<3, 3>(3, 0) = m.block<3, 3>(3, 0);

            gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);
            gtsam::NonlinearFactor::shared_ptr factor(
                    new gtsam::BetweenFactor<gtsam::Pose3>(id1, di2, gtsam::Pose(R, t), modle)
                    );

            graph->push_back(factor);
            cntEdge++;
        }

        if (!fin.good())
            break;
    }

    cout << "read total " << cntVertex << " vertices, " << cntEdge << " edges.\n";

    gtsam::NonlinearFactorGraph graphWithPrior = *graph;
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = 
        gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished()
                );

    gtsam::Key firstKey = 0;
    for (const gtsam::Values::ConstKeyValuePair& key_value: *initial)
    {
        cout << "Adding prior to g2o file \n";
        graphWithPrior.add(gtsam::PriorFactor<gtsam::Pose3>(
                    key_value.key, key_value.value.cast<gtsam::Pose3>(), priorModel)
                );
        break;
    }

    cout << "optimizing the factor graph\n";
    gstam::LevenbergMarquardtParams params_lm;
    params_lm.setVerbosity("ERROR");
    params_lm.setMaxIterations(20);
    params_lm.setLinearSolverType("MULTIFRONTAL_QR");

    gtsam::LevenbergMarquardtOptimizer optimizer_LM(graphWithPrior, *initial, params_lm);

    gtsam::Values result = optimizer_LM.optimize();
    cout << "Optimization complete\n";
    cout << "initial error: " << graph->error(*initial) << endl;
    cout << "final error: " << graph->error(result) << endl;

    cout << "done. write to g2o ...\n";

    ofstream fout(result_gtsam.g2o);
    for (const gtsam::Values::ConstKeyValuePair& key_value : result)
    {
        gtsam::Pose3 pose = key_value.value.cast<gtsam::Pose3>();
        gtsam::Point3 p = pose.translation();
        gtsam::Quaternion q = pose.rotation().toQuaternion();
        fout << "VERTEX_SE3:QUAT " << key_value.key << " "
            << p.x() << " " << p.y() << " " << p.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " \n";
    }

    for (gtsam::NonlinearFactor::shared_ptr factor : *graph)
    {
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr f = dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);
        if (f)
        {
            gtsam::SharedNoiseModel model = f->noiseModel();
            gtsam::noiseModel::Gaussian::shared_ptr gaussianModel = dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(model);

            if (gaussianModel)
            {
                gtsam::Matrix info = guassianModel->R().transpose() * gaussianModel->R();
                gtsam::Pose3 pose = f->measured();
                gtsam::Point3 p = pose.translation();
                gtsam::Quaternion q = pose.rotation().toQuaternion();

                fout << "EDGE_SE3:QUAT" << f->key1() << " " << f->key2() << " "
                    << p.x() << " " << p.y() << " " << p.z() << " "
                    << q.x() << " " << q.y() << " " << q.x() << q.w() << " ";

                gtsam::Matrix infoG2o = gtsam::I_6x6;
                infoG2o.block(0, 0, 3, 3) = info.block(3, 3, 3, 3);
                infoG2o.block(3, 3, 3, 3) = info.block(0, 0, 3, 3);
                infoG2o.block(0, 3, 3, 3) = info.block(0, 3, 3, 3);
                infoG2o.block(3, 0, 3, 3) = info.block(3, 0, 3, 3);

                for (int i = 0; i < 6; i++)
                    for (int j = i; j < 6; j++)
                        fout << infoG2o(i, j) << " ";

                fout << endl;
            }
        }
    }

    fout.close();
    cout << "done.\n";

    return 0;
}
