/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include "render.h"


Car::Car()
    : position(Vect3(0, 0, 0)), dimensions(Vect3(0, 0, 0)), color(Color(0, 0, 0))
{}


Car::Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity, float setAngle, float setLf, std::string setName)
    : position(setPosition), dimensions(setDimensions), color(setColor), velocity(setVelocity), angle(setAngle), Lf(setLf), name(setName)
{
    orientation = getQuaternion(angle);
    acceleration = 0;
    steering = 0;
    accuateIndex = -1;

    sinNegTheta = sin(-angle);
    cosNegTheta = cos(-angle);

    return;
}

// angle around z axis
Eigen::Quaternionf Car::getQuaternion(float theta)
{
    Eigen::Matrix3f rotation_mat;

    rotation_mat << 
        cos(theta), -sin(theta),    0,
        sin(theta), cos(theta),     0,
        0,          0,              1;
    Eigen::Quaternionf q(rotation_mat);

    return q;
}

void Car::render(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // render bottom of car
    viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*1/3), orientation, dimensions.x, dimensions.y, dimensions.z*2/3, name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
    viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*1/3), orientation, dimensions.x, dimensions.y, dimensions.z*2/3, name+"frame");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name+"frame");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name+"frame");

    // render top of car
    viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*5/6), orientation, dimensions.x/2, dimensions.y, dimensions.z*1/3, name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name + "Top");
    viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*5/6), orientation, dimensions.x/2, dimensions.y, dimensions.z*1/3, name + "Topframe");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name+"Topframe");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name+"Topframe");

    return;
}

void Car::setAcceleration(float setAcc)
{
    acceleration = setAcc;
}

void Car::setSteering(float setSteer)
{
    steering = setSteer;
}

void Car::setInstructions(std::vector<accuation> setIn)
{
    for (accuation a : setIn) {
        instructions.push_back(a);
    }
}

void Car::setUKF(UKF tracker)
{
    ukf = tracker;
}

void Car::move(float dt, int time_us)
{
    if ((instructions.size() > 0) && (accuateIndex < ((int)instructions.size() - 1))) {
        if (time_us >= instructions[accuateIndex + 1].time_us) {
            setAcceleration(instructions[accuateIndex + 1].acceleration);
            setSteering(instructions[accuateIndex + 1].steering);
            accuateIndex++;
        }
    }

    position.x += velocity * cos(angle) * dt;
    position.y += velocity * sin(angle) * dt;
    angle += velocity*steering*dt/Lf;
    orientation = getQuaternion(angle);
    velocity += acceleration*dt;

    sinNegTheta = sin(-angle);
    cosNegTheta = cos(-angle);

    return;
}

// collision helper function
bool Car::inbetween(double point, double center, double range)
{
    return (center - range <= point) && (center + range >= point);
}

bool Car::checkCollision(Vect3 point)
{
    // check collision for rotated car
    double xPrime = ((point.x-position.x) * cosNegTheta - (point.y-position.y) * sinNegTheta)+position.x;
    double yPrime = ((point.y-position.y) * cosNegTheta + (point.x-position.x) * sinNegTheta)+position.y;

    return (inbetween(xPrime, position.x, dimensions.x / 2) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
        (inbetween(xPrime, position.x, dimensions.x / 4) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));
}


//------------------------------------------------

void renderHighway(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // units in meters
    double roadWidth = 12.0, roadHeight = 0.2;
    double roadLengthAhead = 50.0, roadLengthBehind = -15.0;

    viewer->addCube(roadLengthBehind, roadLengthAhead, -roadWidth / 2, roadWidth / 2, -roadHeight, 0, .2, .2, .2, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "highwayPavement");
    viewer->addLine(pcl::PointXYZ(roadLengthBehind, -roadWidth / 6, 0.01), pcl::PointXYZ(roadLengthAhead , -roadWidth / 6, 0.01), 1, 1, 0, "line1");
    viewer->addLine(pcl::PointXYZ(roadLengthBehind, roadWidth / 6, 0.01), pcl::PointXYZ(roadLengthAhead, roadWidth / 6, 0.01), 1, 1, 0, "line2");

    // render poles
    // spacing in meters between poles, poles start at x = 0
    double poleSpace = 10;
    // pole distance from road curve
    double poleCurve = 4;
    double poleWidth = 0.5;
    double poleHeight = 3;

    //double distancePos = 7;
    double markerPos = (roadLengthBehind/poleSpace)*poleSpace-distancePos;
    while(markerPos < roadLengthBehind) {
        markerPos += poleSpace;
    }
    int poleIndex = 0;
    while(markerPos <= roadLengthAhead) {
        // left pole
        viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2+roadWidth/2+poleCurve, poleWidth/2+roadWidth/2+poleCurve, 0, poleHeight, 1, 0.5, 0, "pole_"+std::to_string(poleIndex)+"l");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "pole_"+std::to_string(poleIndex)+"l");
        viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2+roadWidth/2+poleCurve, poleWidth/2+roadWidth/2+poleCurve, 0, poleHeight, 0, 0, 0, "pole_"+std::to_string(poleIndex)+"lframe");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "pole_"+std::to_string(poleIndex)+"lframe");
        // right pole
        viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2-roadWidth/2-poleCurve, poleWidth/2-roadWidth/2-poleCurve, 0, poleHeight, 1, 0.5, 0, "pole_"+std::to_string(poleIndex)+"r");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "pole_"+std::to_string(poleIndex)+"r");
        viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2-roadWidth/2-poleCurve, poleWidth/2-roadWidth/2-poleCurve, 0, poleHeight, 0, 0, 0, "pole_"+std::to_string(poleIndex)+"rframe");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "pole_"+std::to_string(poleIndex)+"rframe");

        markerPos += poleSpace;
        poleIndex++;
    }

    return;
}

int countRays = 0;
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    for (pcl::PointXYZ point : cloud->points) {
        viewer->addLine(pcl::PointXYZ(origin.x, origin.y, origin.z), point, 1, 0, 0, "ray" + std::to_string(countRays));
        countRays++;
    }

    return;
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    while (countRays) {
        countRays--;
        viewer->removeShape("ray" + std::to_string(countRays));
    }

    return;
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
{
    viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);

    return;
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color)
{
    if (color.r == -1) {
        // Select color based off of cloud intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
        viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    } else {
        // Select color based off input value
        viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    }

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

    return;
}

// Draw wire frame box with filled transparent color 
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color, float opacity)
{
    if (opacity > 1.0) {
        opacity = 1.0;
    }
    if (opacity < 0.0) {
        opacity = 0.0;
    }

    std::string cube = "box" + std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);

    return;
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity)
{
    if (opacity > 1.0) {
        opacity = 1.0;
    }
    if (opacity < 0.0) {
        opacity = 0.0;
    }
    std::string cube = "box" + std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);

    return;
}


