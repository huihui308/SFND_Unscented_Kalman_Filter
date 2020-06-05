/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#ifndef RENDER_H
#define RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <vector>
#include <string>
#include "../ukf.h"


struct Color
{
    float r, g, b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {}
};

struct Vect3
{
    double x, y, z;

    Vect3(double setX, double setY, double setZ)
        : x(setX), y(setY), z(setZ)
    {}

    Vect3 operator+(const Vect3& vec)
    {
        Vect3 result(x + vec.x, y + vec.y, z + vec.z);
        return result;
    }
};

enum CameraAngle
{
    XY, TopDown, Side, FPS
};

struct accuation
{
    float steering;
    long long time_us;
    float acceleration;

    accuation(long long t, float acc, float s)
        : time_us(t), acceleration(acc), steering(s)
    {}
};

class Car
{
public:
    // units in meters
    Vect3 position, dimensions;
    Eigen::Quaternionf orientation;
    std::string name;
    Color color;
    float velocity;
    float angle;
    float acceleration;
    float steering;
    // distance between front of vehicle and center of gravity
    float Lf;
    UKF ukf;
    //accuation instructions
    std::vector<accuation> instructions;
    int accuateIndex;

    double sinNegTheta;
    double cosNegTheta;

    explicit Car();

    Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity, float setAngle, float setLf, std::string setName);

    // angle around z axis
    Eigen::Quaternionf getQuaternion(float theta);

    void render(pcl::visualization::PCLVisualizer::Ptr& viewer);

    void setAcceleration(float setAcc);

    void setSteering(float setSteer);

    void setInstructions(std::vector<accuation> setIn);

    void setUKF(UKF tracker);

    void move(float dt, int time_us);

    // collision helper function
    bool inbetween(double point, double center, double range);

    bool checkCollision(Vect3 point);
};

void renderHighway(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1, 1, 1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1, -1, -1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1, 0, 0), float opacity = 1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color = Color(1, 0, 0), float opacity = 1);

#endif
