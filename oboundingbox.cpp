#include "oboundingbox.h"

OBoundingBox::OBoundingBox(const TriMesh& _mesh)
    : mesh(_mesh)
{
    cloud = Auxiliary::ToCloud(mesh);
}

OBoundingBox::OBoundingBox(const Cloud& _cloud)
    : cloud(_cloud)
{
}

void OBoundingBox::aquireOBB()
{
    //得到所有顶点
    Eigen::MatrixXf X = Eigen::MatrixXf::Zero(cloud.points.size(), 3);

    for (size_t i = 0; i < cloud.points.size(); i++) {
        pcl::PointXYZ cloud_point = cloud.points.at(i);
        vertex_set.push_back(Eigen::Vector3f(cloud_point.x, cloud_point.y, cloud_point.z));
        X(i, 0) = cloud_point.x;
        X(i, 1) = cloud_point.y;
        X(i, 2) = cloud_point.z;
    }

    float meanx = (X.block(0, 0, X.rows(), 1)).mean();
    float meany = (X.block(0, 1, X.rows(), 1)).mean();
    float meanz = (X.block(0, 2, X.rows(), 1)).mean();

    Eigen::MatrixXf MeanX = Eigen::MatrixXf::Ones(cloud.points.size(), 3);
    MeanX.block(0, 0, MeanX.rows(), 1) *= meanx;
    MeanX.block(0, 1, MeanX.rows(), 1) *= meany;
    MeanX.block(0, 2, MeanX.rows(), 1) *= meanz;

    //去中心
    X = X - MeanX;
    //计算协方差矩阵
    Eigen::MatrixXf COV = (X.transpose() * X) / float(cloud.points.size() - 1);
    //计算特征值和特征向量
    Eigen::EigenSolver<Eigen::MatrixXf> es(COV);
    MatrixXf D = es.pseudoEigenvalueMatrix(); //计算特征值
    MatrixXf V = es.pseudoEigenvectors(); //计算特征向量,每一列是一个特征向量

    /*std::cout <<"******************eigen solve information********************" << std::endl;
    std::cout << "C:\n" << COV << std::endl;
    std::cout << "\neigen value:\n" << D << std::endl;
    std::cout << "\neigen vector:\n" << V << std::endl;
    std::cout <<"\nV*D*V.inverse()\n"<<V*D*V.inverse() << std::endl;*/

    length = TriMesh::Normal(V(0, 0), V(1, 0), V(2, 0)).normalize(); //最长轴
    height = TriMesh::Normal(V(0, 1), V(1, 1), V(2, 1)).normalize(); //次长轴
    width = TriMesh::Normal(V(0, 2), V(1, 2), V(2, 2)).normalize(); //最短轴

    //右手系
    if (OpenMesh::dot(OpenMesh::cross(length, height), width) < 0) {
        width *= -1;
    }

    /*std::cout<<"obb information"<<std::endl;
    std::cout<<"width axis: "<<width<<std::endl;
    std::cout<<"height axis: "<<height<<std::endl;
    std::cout<<"length axis: "<<length<<std::endl;*/

    //求中心点
    Centroid = TriMesh::Point(0, 0, 0); //几何中心
    //计算几何中心
    for (size_t i = 0; i < vertex_set.size(); i++) {
        TriMesh::Point p(vertex_set[i][0], vertex_set[i][1], vertex_set[i][2]);
        Centroid += p;
    }
    Centroid /= vertex_set.size();

    //坐标变换：将所有点从XYZ坐标变换到xyz->LHW坐标，新的坐标原点为几何中心,Centroid
    /* {O,i,j,k}->{Centroid,l,h,w}
     * (x,y,z)=(c1,c2,c3)+(l,h,w)A;
     * (x,y,z)=>(l,h,w)
     * (l,h,w)=[(x,y,z)-(c1,c2,c3)]*inv(A)
    */

    //新坐标原点
    MatrixXf C(1, 3);
    C << Centroid[0], Centroid[1], Centroid[2];
    //变换矩阵
    Matrix3f A;
    A << length[0], length[1], length[2],
        height[0], height[1], height[2],
        width[0], width[1], width[2];

    //std::cout<<"transform matrix:\n"<<A<<std::endl;

    //更新点坐标+找边界(将模型中所有点的坐标转换到LHW坐标)
    //找点云边界
    float minl = std::numeric_limits<float>::max(), maxl = std::numeric_limits<float>::lowest();
    float minh = std::numeric_limits<float>::max(), maxh = std::numeric_limits<float>::lowest();
    float minw = std::numeric_limits<float>::max(), maxw = std::numeric_limits<float>::lowest();
    for (size_t i = 0; i < vertex_set.size(); i++) {
        MatrixXf P(1, 3);
        P << vertex_set[i].x(), vertex_set[i].y(), vertex_set[i].z();
        MatrixXf _P(1, 3);
        _P = (P - C) * A.inverse();

        //_P = A.transpose().inverse() * (P - C);

        vertex_set[i] = Eigen::Vector3f(_P(0, 0), _P(0, 1), _P(0, 2));

        //L轴
        if (vertex_set[i].x() <= minl) {
            minl = vertex_set[i].x();
        }
        if (vertex_set[i].x() >= maxl) {
            maxl = vertex_set[i].x();
        }
        //H轴
        if (vertex_set[i].y() <= minh) {
            minh = vertex_set[i].y();
        }
        if (vertex_set[i].y() >= maxh) {
            maxh = vertex_set[i].y();
        }
        //W轴
        if (vertex_set[i].z() <= minw) {
            minw = vertex_set[i].z();
        }
        if (vertex_set[i].z() >= maxw) {
            maxw = vertex_set[i].z();
        }
    }
    //*************************找边界完成

    //生成8个顶点
    //l轴，前面,从第一象限开始,逆时针(从L轴的头看)
    //l轴，后面,从第一象限开始,逆时针(从L轴的头看)
    vertices.resize(8);
    vertices[0] = TriMesh::Point(maxl, maxh, maxw);
    vertices[1] = TriMesh::Point(maxl, minh, maxw);
    vertices[2] = TriMesh::Point(maxl, minh, minw);
    vertices[3] = TriMesh::Point(maxl, maxh, minw);

    vertices[4] = TriMesh::Point(minl, maxh, maxw);
    vertices[5] = TriMesh::Point(minl, minh, maxw);
    vertices[6] = TriMesh::Point(minl, minh, minw);
    vertices[7] = TriMesh::Point(minl, maxh, minw);

    //将这些顶点转换回XYZ坐标
    //(x, y, z) = (c1, c2, c3) + (l,h, w)A;
    //(l,h,w)=>(x,y,z)
    for (size_t i = 0; i < vertices.size(); i++) {
        MatrixXf P(1, 3);
        P << vertices[i][0], vertices[i][1], vertices[i][2];
        MatrixXf _P(1, 3);

        //_P = C + A.transpose() * P;
        _P = C + P * A;

        vertices[i] = TriMesh::Point(_P(0, 0), _P(0, 1), _P(0, 2));
    }

    //计算边长

    W = distance(vertices[0], vertices[3]);
    H = distance(vertices[0], vertices[1]);
    L = distance(vertices[0], vertices[4]);

    /*std::cout<<"obb information"<<std::endl;
    std::cout<<"width length: "<<W<<std::endl;
    std::cout<<"height length: "<<H<<std::endl;
    std::cout<<"length length: "<<L<<std::endl;*/

    OBBMesh.clear();
    CreateOBBMesh(OBBMesh);

    // write mesh to output.obj
    /*try {
        if (!OpenMesh::IO::write_mesh(OBBMesh, "../obbmesh.off")) {
            std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
        }
    } catch (std::exception& x) {
        std::cerr << x.what() << std::endl;
    }*/
}

void OBoundingBox::getOBBPolyMesh(PolyMesh& OBBMesh)
{
    OBBMesh = this->OBBMesh;
}

void OBoundingBox::getVertices(vector<TriMesh::Point>& vertices)
{
    vertices = this->vertices;
}
void OBoundingBox::getAxis(TriMesh::Normal& length, TriMesh::Normal& height, TriMesh::Normal& width)
{
    length = this->length;
    height = this->height;
    width = this->width;
}

void OBoundingBox::getLengthAxis(TriMesh::Normal& length)
{
    length = this->length;
}

void OBoundingBox::getHeightAxis(TriMesh::Normal& height)
{
    height = this->height;
}

void OBoundingBox::getWidthAxis(TriMesh::Normal& width)
{
    width = this->width;
}

float OBoundingBox::getLength()
{
    return L;
}

float OBoundingBox::getWidth()
{
    return W;
}

float OBoundingBox::getHeight()
{
    return H;
}

TriMesh::Point OBoundingBox::getCentroid()
{
    return Centroid;
}

TriMesh::Point& OBoundingBox::getClosedPoint(TriMesh::Point p)
{
}

TriMesh::Point& OBoundingBox::getClosedPoint(float x, float y, float z)
{
}

void OBoundingBox::CreateOBBMesh(PolyMesh& obbmesh)
{
    PolyMesh::VertexHandle vhandle[8];
    for (size_t i = 0; i < 8; i++) {
        vhandle[i] = obbmesh.add_vertex(vertices[i]);
    }

    // generate (quadrilateral) faces
    std::vector<PolyMesh::VertexHandle> face_vhandles;
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[3]);
    obbmesh.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[4]);
    obbmesh.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[6]);
    obbmesh.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[5]);
    obbmesh.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[4]);
    obbmesh.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[4]);
    obbmesh.add_face(face_vhandles);
}

float OBoundingBox::distance(TriMesh::Point p1, TriMesh::Point p2)
{
    return (float)sqrt(pow(p2[0] - p1[0], 2) + pow(p2[0] - p1[0], 2) + pow(p2[0] - p1[0], 2));
}
