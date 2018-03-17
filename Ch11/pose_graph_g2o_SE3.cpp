#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
using namespace std;

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o\n";
        return 1;
    }

    ifstream fin(argv[1]);
    if (!fin)
    {
        cout << "file " << argv[1] << " does not exist\n";
        return 1;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> > Block;

    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    int vertexCnt = 0, edgeCnt = 0;
    while (!fin.eof()) {
        string name;

        fin >> name;

        if (name == "VERTEX_SE3:QUAT")
        {
            g2o::VertexSE3* v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);

            optimizer.addVertex(v);
            vertexCnt++;

            if (index == 0)
                v->setFixed(true);
        }
        else if (name == "EDGE_SE3:QUAT")
        {
            g2o::EdgeSE3* e = new g2o::EdgeSE3();
            int idx1, idx2;
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);

            optimizer.addEdge(e);
        }

        if (!fin.good())
            break;
    }

    cout << "read total " << vertexCnt << "vertices, " << edgeCnt << "edges.\n";

    cout << "prepare optimizing ...\n";
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    
    cout << "calling optimizing ...\n";
    optimizer.optimize(30);

    cout << "saving optimization results ...\n";
    optimizer.save("result.g2o");

    return 0;
}
