/*
 * @Description: 获取任意流形网格的包围盒
 * @Author: lydia
 * @LastEditors: Please set LastEditors
 * @Date: 2019-04-07 19:30:59
 * @LastEditTime: 2019-04-08 17:03:08
 * @usage:
 * OBoundingBox *obb=new OBoundingBox(mesh);
 * obb.aquireOBB();
 * obb.getPolyMesh(PolyMesh &polymesh);
 * obb.getVertices();
 * obb.getCentroid();
 * obb.getAxis(TriMesh::Normal &length,TriMesh::Normal &height,TriMesh::Normal &width);
 */

#ifndef OBOUNDINGBOX_H
#define OBOUNDINGBOX_H

//local
#include "CGAuxFunc/globaldefinition.h"
#include "CGAuxFunc/auxiliary.h"

//eigen
#include <Eigen/Eigen>
//std
#include <vector>
#include <math.h>

using namespace std;
using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::Vector3f;
using Eigen::VectorXf;

class OBoundingBox {
public:
    OBoundingBox(const TriMesh& _mesh);
    OBoundingBox(const Cloud& _cloud);

    void aquireOBB();
    void getOBBPolyMesh(PolyMesh& OBBMesh);

    void getVertices(vector<TriMesh::Point>& vertices); //顶点

    void getAxis(TriMesh::Normal& length, TriMesh::Normal& height, TriMesh::Normal& width); //坐标轴

    /**
     * @description: 获得最长的边的单位向量
     * @author: lydia
     * @param {TriMesh::Normal &} 
     * @return: viod
     * @Date: 2019-04-07 19:35:26
     */
    void getLengthAxis(TriMesh::Normal& length);
    /**
     * @description: 获取次长的边的单位向量
     * @author: lydia
     * @param {TriMesh::Normal &} 
     * @return: 
     * @Date: 2019-04-07 19:38:46
     */
    void getHeightAxis(TriMesh::Normal& height);
    /**
     * @description: 获取最短的边的单位向量
     * @author: lydia
     * @param {TriMesh::Normal&} 
     * @return: 
     * @Date: 2019-04-07 19:39:27
     */
    void getWidthAxis(TriMesh::Normal& width);

    /**
     * @description: 获取最长的边的长度
     * @author: lydia
     * @param {void} 
     * @return: 
     * @Date: 2019-04-07 19:39:56
     */
    float getLength();
    /**
     * @description: 获取次长的边的长度
     * @author: lydia
     * @param {void} 
     * @return: 
     * @Date: 2019-04-07 19:40:22
     */
    float getHeight();
    /**
     * @description: 获取最短的边的长度
     * @author: lydia
     * @param {void} 
     * @return: 
     * @Date: 2019-04-07 19:40:49
     */
    float getWidth();

    /**
     * @description: 获取中心点全局坐标
     * @author: lydia
     * @param {type} 
     * @return: 
     * @Date: 2019-04-07 19:43:44
     */
    TriMesh::Point getCentroid(); //中心

    ~OBoundingBox();

public:
    //helper
    //得到距离点最近的顶点
    TriMesh::Point& getClosedPoint(TriMesh::Point p);
    TriMesh::Point& getClosedPoint(float x, float y, float z);

private:
    //helper
    void CreateOBBMesh(PolyMesh& obbmesh);
    float distance(TriMesh::Point p1, TriMesh::Point p2);

private:
    //八个顶点,L>0:从第一象限开始,逆时针（从L往下看）
    //L<0,从第一象限开始,逆时针（从L往下看）
    vector<TriMesh::Point> vertices;
    //三条轴，length:长，最长的，width:宽,最短的，height:高，中间长度
    TriMesh::Normal length, width, height;
    //几何中心
    TriMesh::Point Centroid;
    //OBB的边长,长宽高
    float L, W, H;

    TriMesh mesh;
    Cloud cloud;

    //PolyMesh:OBBMesh
    PolyMesh OBBMesh;

    //所有顶点，用来做PCA
    vector<Eigen::Vector3f> vertex_set;
};

#endif // OBOUNDINGBOX_H
