#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "renderer.h"
#include <iostream>
#include <fstream>

void loadInput(vector<Vector3f> &vectorVertex,
               vector<Vector3f> &vectorNormal,
               vector<vector<unsigned>> &vectorFace,
               string filePath) {
    ifstream inFile;
    inFile.open(filePath);
    if (!inFile) {
        cerr << "Unable to open obj file";
        exit(1);
    }
    string line;
    while (getline(inFile, line)) {
        std::stringstream ss(line);
        std::string s;
        ss >> s;
        if (s == "v") {
            Vector3f v;
            ss >> v[0] >> v[1] >> v[2];
            vectorVertex.push_back(v);
        } else if (s == "vn") {
            Vector3f v;
            ss >> v[0] >> v[1] >> v[2];
            vectorNormal.push_back(v);
        } else if (s == "f") {
            vector<string> rawVVector;
            string rawV;
            for (int i = 0; i < 3; ++i) {
                ss >> rawV;
                rawVVector.push_back(rawV);
            }
            vector<unsigned> indexVec;
            for (string rawV : rawVVector) {
                stringstream ssInner(rawV);
                string curResult;
                while (getline(ssInner, curResult, '/')) {
                    stringstream ssInt(curResult);
                    int curInt;
                    ssInt >> curInt;
                    indexVec.push_back(curInt);
                }
            }
            vectorFace.push_back(indexVec);
        }
    }
}

int main(int argc, char **argv) {
    assert(argc > 1);

    // 构建相机
    Vector3f eyePoint(6, 0, 0);
    Vector3f viewDirection(-1, 0, 0);
    Vector3f upDirection(0, 0, -1);

    Camera camera(eyePoint, viewDirection, upDirection, -1, 1, -1, 1, -5, -6);

    // 构建光源
    LightSource lightSource;
    Vector3f lightSourceDirection(1, 0.7, 0.7);
    lightSourceDirection.normalize();
    lightSource.intensity = 1;
    lightSource.direction = lightSourceDirection;

    // 构建场景
    Scene scene;
    Vector3f backgroundColor(0, 0, 0);
    scene.ambientLightIntensity = 1;
    scene.backgroundColor = backgroundColor;
    scene.lightSources.push_back(&lightSource);

    Vector3f ambientColor(0.5, 0, 0);
    Vector3f diffuseColor(0.5, 0, 0);
    Vector3f specularColor(0.2, 0.2, 0.2);
    Vector3f mirrorColor(0, 0, 0);

    vector<Vector3f> vectorVertex;
    vector<Vector3f> vectorNormal;
    vector<vector<unsigned>> vectorFace;
    loadInput(vectorVertex, vectorNormal, vectorFace, argv[1]);
    for (size_t index = 0; index < vectorFace.size(); ++index) {
        auto face = vectorFace[index];
        Triangle *curTriangle = new Triangle(vectorVertex[face[0] - 1],
                                             vectorVertex[face[3] - 1],
                                             vectorVertex[face[6] - 1],
                                             vectorNormal[face[2] - 1],
                                             vectorNormal[face[5] - 1],
                                             vectorNormal[face[8] - 1],
                                             diffuseColor,
                                             specularColor,
                                             ambientColor,
                                             2);
        scene.surfaces.push_back(curTriangle);
    }

    // 构建渲染器
    Renderer renderer(scene, camera);
    cv::Mat mat(800, 800, CV_32FC3);
    renderer.render(800, 800, mat);

    cv::namedWindow("Display", cv::WINDOW_NORMAL);
    cv::imshow("Display", mat);
    cv::waitKey(0);

    for (Triangle *surface : scene.surfaces) {
        delete surface;
    }
}
