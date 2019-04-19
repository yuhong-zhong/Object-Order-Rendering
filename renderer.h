//
// Created by Yuhong Zhong on 2019-04-18.
//

#ifndef OBJECT_ORDER_RENDERING_RENDERER_H
#define OBJECT_ORDER_RENDERING_RENDERER_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

struct BoundingBox {
    Vector2f minPoint;  // 约束区域靠近原点的顶点
    Vector2f maxPoint;  // 约束区域远离原点的顶点
};

class Triangle {
public:
    // color
    Vector3f diffuseColor;
    Vector3f specularColor;
    Vector3f ambientColor;
    float specularParameter;
    // geometry
    Vector4f vertices[3];
    Vector3f normals[3];
    // vertex processed value
    Vector3f colors[3];
    Vector4f tmpVertices[3];

    Triangle(Vector3f &v0, Vector3f &v1, Vector3f &v2,
             Vector3f &n0, Vector3f &n1, Vector3f &n2,
             Vector3f &diffuseColor, Vector3f &specularColor,
             Vector3f &ambientColor, float specularParameter);

    Triangle(Triangle &srcTriangle, bool useTmp = false);

    void boundingBox(BoundingBox &box);
};

class Camera {
public:
    Vector3f eyePoint;
    Vector3f u, v, w;
    float l, r, b, t, n, f;

    Camera(Vector3f &eyePoint, Vector3f &u, Vector3f &v, Vector3f &w,
           float l, float r, float b, float t, float n, float f);

    Camera(Vector3f &eyePoint, Vector3f &viewDirection, Vector3f &upDirection,
           float l, float r, float b, float t, float n, float f);

    Camera();
};

class LightSource {
public:
    Vector3f direction;
    float intensity;
};

class Scene {
public:
    vector<Triangle *> surfaces;
    vector<Triangle *> processedSurfaces;

    vector<LightSource *> lightSources;
    float ambientLightIntensity;
    Vector3f backgroundColor;
};


class Renderer {
private:
    Scene &scene;
    Camera &camera;
    MatrixXf *matrixR;
    MatrixXf *matrixG;
    MatrixXf *matrixB;
    MatrixXf *matrixZ;
    size_t verticalPixel;
    size_t horizontalPixel;
public:
    Renderer(Scene &s, Camera &c);

    void render(size_t verticalPixel, size_t horizontalPixel, cv::Mat &mat);

    void vertexProcess();

    void rasterization();
};


#endif //OBJECT_ORDER_RENDERING_RENDERER_H
