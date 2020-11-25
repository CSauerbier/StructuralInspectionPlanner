#include "Sampling/VPsampling.h"


StateVector RandomSampling::getVP(tri_t* tri)
{
    StateVector vp;
    Eigen::Vector3f pos_extended, pos_on_tri, vp_pos;

    //Random distance to the triangle within the constraints
    float dist = randNum(tri->minDist, tri->maxDist);

    std::array<Eigen::Vector3f, 3> vertices, normed_boundaries, boundary_points;

    vertices[0] = tri->x1;
    vertices[1] = tri->x2;
    vertices[2] = tri->x3;
    
    //Array to hold normalized vectors that delimit the sampling space angular boundaries
    normed_boundaries[0] = (tri->n1.cross(tri->n2)).normalized();
    normed_boundaries[1] = (tri->n2.cross(tri->n3)).normalized();
    normed_boundaries[2] = (tri->n3.cross(tri->n1)).normalized();

    //Extend normalized boundary vectors onto the hyperplane parallel to the triangle and at a distance of 1
    for(auto i=0; i<vertices.size(); i++)
    {
        //Dot product proportional to cosine
        float hypotenuse = 1/(normed_boundaries[i].dot(tri->aabs));
        boundary_points[i] = vertices[i] + normed_boundaries[i]*hypotenuse;
    }

    //Sample random percentages at how far the new position will be moved into v1-v0 and v2-v1 direction respectively
    //Results in points randomly sampled on a rhomboid. Only accept samples that are on one half of the rhomboid that is the triangle
    std::array<float,2> weights;
    do
    {
        weights[0] = randNum();
        weights[1] = randNum();
        
    } while (weights[0] < weights[1]);
    pos_extended = boundary_points[0] + (boundary_points[1]-boundary_points[0])*weights[0] + (boundary_points[2]-boundary_points[1])*weights[1];
    pos_on_tri = vertices[0] + (vertices[1]-vertices[0])*weights[0] + (vertices[2]-vertices[1])*weights[1];

    //Extend the sampled point to be at the correct distance to the triangle
    vp_pos = pos_on_tri + ((pos_extended - pos_on_tri).normalized())*dist;

    vp[0] = vp_pos[0];
    vp[1] = vp_pos[1];
    vp[2] = vp_pos[2];

    //Setting pitch entry so that the optical axis vector points at the center of the triangle
    Vector3f tri_mean((tri->x1[0]+tri->x2[0]+tri->x3[0])/3,
                     (tri->x1[1]+tri->x2[1]+tri->x3[1])/3,
                     (tri->x1[2]+tri->x2[2]+tri->x3[2])/3  );
    vp[3] = atan2(tri_mean[1]-vp[1], tri_mean[0]-vp[0]);
    vp[4] = atan2(vp[2]-tri_mean[2], hypot(vp[0]-tri_mean[0], vp[1]-tri_mean[1]));

    return vp;
}

float RandomSampling::randNum(float low, float high)
{
    float rand_norm = randNum();
    float diff = high-low;
    return low + rand_norm*diff;
}

float RandomSampling::randNum()
{
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0.0, 1.0);
    return dis(gen);
}


EquidistantPointsOnSphere::EquidistantPointsOnSphere(std::vector<double> center, float radius, unsigned int n_target)
{
    this->counter = 0;
    this->samplePositionsUnitSphere(n_target);
    this->convertToStateVector(center, radius);
}

EquidistantPointsOnSphere::~EquidistantPointsOnSphere()
{
    for(auto vp: view_points)
    {
        delete vp;
    }
    
    for(auto point: points)
    {
        delete point;
    }
}

/**
 * Algorithm according to Deserno 2004
 * */
void EquidistantPointsOnSphere::samplePositionsUnitSphere(unsigned int n_target)
{
    unsigned int n_count = 0;
    float a = 4*M_PI/n_target;
    float d = sqrt(a);
    unsigned int m_theta = std::round((float)M_PI/d);
    float d_theta = M_PI/m_theta;
    float d_phi = a/d_theta;

    for(size_t m=0; m<m_theta; m++)
    {
        float theta = M_PI*(m + 0.5)/m_theta;
        unsigned int m_phi = std::round((float)(2*M_PI*sin(theta)/d_phi));
        
        for(size_t n=0; n<m_phi; n++)
        {
            float phi = 2*M_PI*n/m_phi;

            float x = sin(theta)*cos(phi);
            float y = sin(theta)*sin(phi);
            float z = cos(theta);
            Eigen::Vector3f* tmp = new Eigen::Vector3f(x,y,z);
            points.push_back(tmp);

            n_count++;
        }
    }
}

void EquidistantPointsOnSphere::convertToStateVector(std::vector<double> center, float radius)
{
    auto test = points.at(0)->coeff(0);
    for(auto point: points)
    {
        float x = radius*point->coeff(0) + center[0];
        float y = radius*point->coeff(1) + center[1];
        float z = radius*point->coeff(2) + center[2];
        float yaw = std::atan2(-point->coeff(1), -point->coeff(0));
        float pitch = std::atan2(point->coeff(2), hypot(point->coeff(0), point->coeff(1)));
        StateVector* tmp = new StateVector;
        *tmp << x, y, z, yaw, pitch;
        view_points.push_back(tmp);
    }
}

StateVector* EquidistantPointsOnSphere::getVP()
{
    if(view_points.size() > counter) counter++;
    return view_points.at(counter-1);
}

size_t EquidistantPointsOnSphere::numberOfPointsGenerated()
{
    return this->view_points.size();
}