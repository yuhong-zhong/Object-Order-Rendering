//
// Created by Yuhong Zhong on 2019-04-18.
//

#include "renderer.h"
#include "utils.h"


Triangle::Triangle(Vector3f &v0, Vector3f &v1, Vector3f &v2,
                   Vector3f &n0, Vector3f &n1, Vector3f &n2,
                   Vector3f &diffuseColor, Vector3f &specularColor,
                   Vector3f &ambientColor, float specularParameter) {
    this->vertices[0].block<3, 1>(0, 0) = v0;
    this->vertices[1].block<3, 1>(0, 0) = v1;
    this->vertices[2].block<3, 1>(0, 0) = v2;
    this->vertices[0](3) = 1;
    this->vertices[1](3) = 1;
    this->vertices[2](3) = 1;
    this->normals[0] = n0.normalized();
    this->normals[1] = n1.normalized();
    this->normals[2] = n2.normalized();
    this->diffuseColor = diffuseColor;
    this->specularColor = specularColor;
    this->ambientColor = ambientColor;
    this->specularParameter = specularParameter;
}

Triangle::Triangle(Triangle &srcTriangle, bool useTmp) {
    if (useTmp) {
        for (size_t vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            this->vertices[vertexIndex] = srcTriangle.tmpVertices[vertexIndex];
        }
    } else {
        for (size_t vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            this->vertices[vertexIndex] = srcTriangle.vertices[vertexIndex];
        }
    }

    for (size_t vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
        this->normals[vertexIndex] = srcTriangle.normals[vertexIndex];
        this->colors[vertexIndex] = srcTriangle.colors[vertexIndex];
    }
    this->diffuseColor = srcTriangle.diffuseColor;
    this->specularColor = srcTriangle.specularColor;
    this->ambientColor = srcTriangle.ambientColor;
    this->specularParameter = srcTriangle.specularParameter;
}

void Triangle::boundingBox(BoundingBox &box) {
    for (int i = 0; i < 2; ++i) {
        box.maxPoint(i) = max({this->vertices[0][i], this->vertices[1][i], this->vertices[2][i]});
        box.minPoint(i) = min({this->vertices[0][i], this->vertices[1][i], this->vertices[2][i]});
    }
}

Camera::Camera() {
    this->eyePoint.setConstant(0);
    this->u.setConstant(0);
    this->v.setConstant(0);
    this->w.setConstant(0);
    this->u(0) = 1;
    this->v(2) = 1;
    this->w(1) = -1;
    this->l = -1;
    this->r = 1;
    this->b = -1;
    this->t = 1;
    this->n = -1;
    this->f = -2;
}

Camera::Camera(Vector3f &eyePoint, Vector3f &viewDirection, Vector3f &upDirection,
               float l, float r, float b, float t, float n, float f) {
    this->eyePoint = eyePoint;
    this->w = -1 * viewDirection.normalized();
    this->u = (upDirection.cross(this->w)).normalized();
    this->v = this->w.cross(this->u);
    this->l = l;
    this->r = r;
    this->b = b;
    this->t = t;
    this->n = n;
    this->f = f;
}

Camera::Camera(Vector3f &eyePoint, Vector3f &u, Vector3f &v, Vector3f &w,
               float l, float r, float b, float t, float n, float f) {
    this->eyePoint = eyePoint;
    this->u = u;
    this->v = v;
    this->w = w;
    this->l = l;
    this->r = r;
    this->b = b;
    this->t = t;
    this->n = n;
    this->f = f;
}

Renderer::Renderer(Scene &s, Camera &c) : scene(s), camera(c) {
    ;
}

void Renderer::render(size_t verticalPixel, size_t horizontalPixel, cv::Mat &mat) {
    MatrixXf matrixR(verticalPixel, horizontalPixel);
    MatrixXf matrixG(verticalPixel, horizontalPixel);
    MatrixXf matrixB(verticalPixel, horizontalPixel);
    MatrixXf matrixZ(verticalPixel, horizontalPixel);
    this->matrixR = &matrixR;
    this->matrixG = &matrixG;
    this->matrixB = &matrixB;
    this->matrixZ = &matrixZ;
    this->verticalPixel = verticalPixel;
    this->horizontalPixel = horizontalPixel;

    matrixZ.setConstant(1);
    matrixR.setConstant(this->scene.backgroundColor(0));
    matrixG.setConstant(this->scene.backgroundColor(1));
    matrixB.setConstant(this->scene.backgroundColor(2));

    vertexProcess();
    rasterization();

    cv::Mat matR(verticalPixel, horizontalPixel, CV_32FC1);
    cv::Mat matG(verticalPixel, horizontalPixel, CV_32FC1);
    cv::Mat matB(verticalPixel, horizontalPixel, CV_32FC1);
    std::vector<cv::Mat> matVector;
    matVector.push_back(matR);
    matVector.push_back(matG);
    matVector.push_back(matB);
    cv::eigen2cv(matrixR, matVector[2]);
    cv::eigen2cv(matrixG, matVector[1]);
    cv::eigen2cv(matrixB, matVector[0]);
    cv::merge(matVector, mat);

    for (Triangle *triangleP : this->scene.processedSurfaces) {
        delete triangleP;
    }
}

void Renderer::vertexProcess() {
    // 1. eye transform & camera transform & perspective transform & colorization
    //   1.1 form transform matrix
    Matrix4f eyeTransform;
    eyeTransform.setConstant(0);
    eyeTransform.diagonal() = Vector4f::Ones();
    eyeTransform.block<3, 1>(0, 3) = -1 * this->camera.eyePoint;

    Matrix4f cameraTransform;
    cameraTransform.setConstant(0);
    cameraTransform.block<1, 3>(0, 0) = this->camera.u.transpose();
    cameraTransform.block<1, 3>(1, 0) = this->camera.v.transpose();
    cameraTransform.block<1, 3>(2, 0) = this->camera.w.transpose();
    cameraTransform(3, 3) = 1;

    Matrix4f rawPerspectiveTransform;
    rawPerspectiveTransform.setConstant(0);
    rawPerspectiveTransform(0, 0) = this->camera.n;
    rawPerspectiveTransform(1, 1) = this->camera.n;
    rawPerspectiveTransform(2, 2) = this->camera.n + this->camera.f;
    rawPerspectiveTransform(3, 2) = 1;
    rawPerspectiveTransform(2, 3) = -1 * this->camera.f * this->camera.n;

    Matrix4f orthographicTransform;
    orthographicTransform.setConstant(0);
    orthographicTransform(0, 0) = 2 / (this->camera.r - this->camera.l);
    orthographicTransform(1, 1) = 2 / (this->camera.t - this->camera.b);
    orthographicTransform(2, 2) = 2 / (this->camera.n - this->camera.f);
    orthographicTransform(3, 3) = 1;
    orthographicTransform(0, 3) = -1 * ((this->camera.r + this->camera.l)
                                        / (this->camera.r - this->camera.l));
    orthographicTransform(1, 3) = -1 * ((this->camera.t + this->camera.b)
                                        / (this->camera.t - this->camera.b));
    orthographicTransform(2, 3) = -1 * ((this->camera.n + this->camera.f)
                                        / (this->camera.n - this->camera.f));

    Matrix4f compositeTransform1 = orthographicTransform * rawPerspectiveTransform * cameraTransform * eyeTransform;
    //   1.2 transform triangles & colorization
    for (Triangle *triangleP : this->scene.surfaces) {
        for (size_t vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            // 1.2.1 compute color
            Vector3f color = this->scene.ambientLightIntensity * triangleP->ambientColor;

            for (auto lightSource : this->scene.lightSources) {
                // diffuse color
                color += triangleP->diffuseColor * lightSource->intensity
                         * max(static_cast<float>(0),
                               static_cast<float>(triangleP->normals[vertexIndex].dot(lightSource->direction * -1)));
                // specular color
                Vector3f direction = triangleP->vertices[vertexIndex].head<3>() - this->camera.eyePoint;
                direction.normalize();
                Vector3f h = ((-1 * direction) + (-1 * lightSource->direction)).normalized();
                color += triangleP->specularColor * lightSource->intensity
                         * pow(std::max(static_cast<float>(0),
                                        static_cast<float>(triangleP->normals[vertexIndex].dot(h))),
                               triangleP->specularParameter);
            }
            triangleP->colors[vertexIndex] = color;
            // 1.2.2 computer transformed vertex
            triangleP->tmpVertices[vertexIndex] = compositeTransform1 * triangleP->vertices[vertexIndex];
        }
    }

    // 2. clip (dummy)
    for (Triangle *triangleP : this->scene.surfaces) {
        this->scene.processedSurfaces.push_back(new Triangle(*triangleP, true));

    }

    // 3. viewport transform & colorization & divide w
    Matrix4f viewportTransform;
    viewportTransform.setConstant(0);
    viewportTransform(0, 0) = static_cast<float>(this->horizontalPixel) / 2;
    viewportTransform(1, 1) = static_cast<float>(this->verticalPixel) / 2;
    viewportTransform(2, 2) = 1;
    viewportTransform(3, 3) = 1;
    viewportTransform(0, 3) = static_cast<float>(this->horizontalPixel - 1) / 2;
    viewportTransform(1, 3) = static_cast<float>(this->verticalPixel - 1) / 2;

    for (Triangle *triangleP : this->scene.processedSurfaces) {
        for (size_t vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            triangleP->vertices[vertexIndex] = viewportTransform * triangleP->vertices[vertexIndex];
            triangleP->vertices[vertexIndex] /= triangleP->vertices[vertexIndex](3);
        }
    }
}

void Renderer::rasterization() {
    Vector2f outOfScreenPoint(-1 / static_cast<float>(2 * this->verticalPixel), this->verticalPixel);

    for (Triangle *triangleP : this->scene.processedSurfaces) {
        BoundingBox box;
        triangleP->boundingBox(box);
        size_t xHead = ceil(box.minPoint(0));
        size_t xTail = static_cast<int>(box.maxPoint(0));
        size_t yHead = ceil(box.minPoint(1));
        size_t yTail = static_cast<int>(box.maxPoint(1));

        for (size_t x = xHead; x <= xTail; ++x) {
            for (size_t y = yHead; y <= yTail; ++y) {
                bool flag = true;
                Vector2f positionVector(x, y);

                // check whether the current point is within the triangle
                for (size_t sideIndex = 0; sideIndex < 3; ++sideIndex) {
                    Vector2f sideNormal = getOrthogonal((triangleP->vertices[(sideIndex + 1) % 3] -
                                                         triangleP->vertices[(sideIndex + 2) % 3]).head<2>());

                    float discriminantP = (positionVector -
                                           (triangleP->vertices[(sideIndex + 1) % 3]).head<2>()).dot(sideNormal);
                    float discriminantV = ((triangleP->vertices[sideIndex]).head<2>() -
                                           (triangleP->vertices[(sideIndex + 1) % 3]).head<2>()).dot(sideNormal);

                    if (discriminantP * discriminantV < 0) {
                        flag = false;
                        break;
                    } else if (discriminantV == 0) {
                        flag = false;
                        break;
                    } else if (discriminantP == 0) {
                        float discriminantO = (outOfScreenPoint -
                                               (triangleP->vertices[(sideIndex + 1) % 3]).head<2>()).dot(sideNormal);
                        if (discriminantO * discriminantV < 0) {
                            flag = false;
                            break;
                        }
                    }
                }

                if (!flag) {
                    continue;
                } else {
                    // interpolate color and depth
                    Vector2f acNormal = getOrthogonal((triangleP->vertices[0] - triangleP->vertices[2]).head<2>());
                    Vector2f abNormal = getOrthogonal((triangleP->vertices[0] - triangleP->vertices[1]).head<2>());
                    float beta = ((positionVector - triangleP->vertices[0].head<2>()).dot(acNormal))
                                 / ((triangleP->vertices[1] - triangleP->vertices[0]).head<2>().dot(acNormal));
                    float gamma = ((positionVector - triangleP->vertices[0].head<2>()).dot(abNormal))
                                  / ((triangleP->vertices[2] - triangleP->vertices[0]).head<2>().dot(abNormal));
                    float alpha = 1 - beta - gamma;
                    Vector3f color = (alpha * triangleP->colors[0]) + (beta * triangleP->colors[1]) +
                                     (gamma * triangleP->colors[2]);
                    float depth = (alpha * triangleP->vertices[0](2)) + (beta * triangleP->vertices[1](2)) +
                                  (gamma * triangleP->vertices[2](2));
                    if (depth < (*this->matrixZ)(x, y)) {
                        (*this->matrixR)(x, y) = color(0);
                        (*this->matrixG)(x, y) = color(1);
                        (*this->matrixB)(x, y) = color(2);
                        (*this->matrixZ)(x, y) = depth;
                    }
                }
            }
        }
    }
}
