#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
#include <vector>
#include "Culling/Coordinates.hpp"
#include <math.h>
#include <stdexcept>

#define det(u_x, u_y, v_x, v_y) (u_x*v_y-u_y*v_x)

__device__ const float EPSILON = 0.000001;
__device__ const float TOLERANCE = 0.001;    //TO-DO


bool g_use_gpu;
bool g_is_initialized = false;

const unsigned int THREADS_PER_BLOCK = 512;
unsigned int GRID_SIZE;
unsigned int KERNEL_COUNT;

float3 *d_viewpoints;
float3 *d_test_points;
float3 *d_vertex0, *d_vertex1, *d_vertex2;
bool *d_outputs;
int g_vp_size;
int g_mesh_size;
int g_vertices_size;

float3 *h_viewpoints;
float3 *h_test_points;
float3 *h_vertex0, *h_vertex1, *h_vertex2;
bool *h_outputs;

//Cuda Error Handler
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

//Overloading operators and defining mathematical functions for CUDA float3
//Vector addition
__host__ __device__ float3 operator+(const float3 &a, const float3 &b) 
{
    return make_float3(a.x+b.x, a.y+b.y, a.z+b.z);
}

//Vector subtraction
__host__ __device__ float3 operator-(const float3 &a, const float3 &b) 
{
    return make_float3(a.x-b.x, a.y-b.y, a.z-b.z);
}

//Scalar-Vector multiplication
__host__ __device__ float3 operator*(const float &a, const float3 &b)
{
    return make_float3(a*b.x, a*b.y, a*b.z);
}

//Dot product
__host__ __device__ float dot(const float3 &a, const float3 &b) 
{
    return (a.x*b.x + a.y*b.y + a.z*b.z);
}

//Cross product
__host__ __device__ float3 cross(const float3 &a, const float3 &b) 
{
    return make_float3( a.y*b.z - a.z*b.y, 
                        a.z*b.x - a.x*b.z, 
                        a.x*b.y - a.y*b.x);
}

//Euclidian vector norm (Length of vector)
__host__ __device__ float norm(const float3 &a)
{
    return sqrt(dot(a,a));
}


/**
 * Checks whether the point tested lies within the triangles passed.
 * Algorithm according to https://mathworld.wolfram.com/TriangleInterior.html
 * */
__global__ void within_triangle_query(float *p_x,
                                        float *p_y,
                                        float *v0_x, 
                                        float *v0_y,
                                        float *v1_x, 
                                        float *v1_y, 
                                        float *v2_x, 
                                        float *v2_y, 
                                        bool *output)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    float d1_x = v1_x[i] - v0_x[i];
    float d1_y = v1_y[i] - v0_y[i];
    float d2_x = v2_x[i] - v0_x[i];
    float d2_y = v2_y[i] - v0_y[i];
    
    float denominator = det(d1_x,d1_y,d2_x,d2_y);
    float a = (det(p_x[0],p_y[0],d2_x,d2_y) - det(v0_x[i],v0_y[i],d2_x,d2_y)) / denominator;
    float b = - (det(p_x[0],p_y[0],d1_x,d1_y) - det(v0_x[i],v0_y[i],d1_x,d1_y)) / denominator;
    
    if(a>0 && b>0 && a+b<1) output[i] = true;
    else output[i] = false;
}

/**
 * CPU-Version of CUDA-Kernel-Code to assess performance
 * */
void within_triangle_query_cpu(float p_x,
    float p_y,
    float *v0_x, 
    float *v0_y,
    float *v1_x, 
    float *v1_y, 
    float *v2_x, 
    float *v2_y, 
    bool *output,
    int size)
{
    for (int i=0; i<size; i++)
    {
        float d1_x = v1_x[i] - v0_x[i];
        float d1_y = v1_y[i] - v0_y[i];
        float d2_x = v2_x[i] - v0_x[i];
        float d2_y = v2_y[i] - v0_y[i];

        float denominator = det(d1_x,d1_y,d2_x,d2_y);
        float a = (det(p_x,p_y,d2_x,d2_y) - det(v0_x[i],v0_y[i],d2_x,d2_y)) / denominator;
        float b = - (det(p_x,p_y,d1_x,d1_y) - det(v0_x[i],v0_y[i],d1_x,d1_y)) / denominator;

        if(a>0 && b>0 && a+b<1) output[i] = true;
        else output[i] = false;
    }
}

/**
 * Allocates GPU-Memory for input arrays and copies their contents to the device. Launches CUDA-Kernel 
 * that determines whether the tested points lie within the triangle or not. Results are copied back into 
 * the output array
 * \param num Size of input arrays
 * \param threads Threads per block parameter for CUDA-Kernel launch
 * \param p Coordinates of point being tested for occlusion
 * \param v Arrays of x and y coordinates of the three vertices defining each triangle
 * \param output Array in which the computation results are written. Boolean array of size num must be allocated. 
 * \param use_gpu Specify whether the CUDA-GPU-API or a fallback CPU implementation is to be used
 * */
int *within_triangle_query_gpu_driver(int num,
    float p_x,
    float p_y,
    float *v0_x, 
    float *v0_y,
    float *v1_x, 
    float *v1_y, 
    float *v2_x, 
    float *v2_y, 
    bool *output,
    bool g_use_gpu) 
{
    if(g_use_gpu)
    {
        // device copies of inputs and output
        float *d_p_x, *d_p_y;
        float *d_v0_x, *d_v0_y, *d_v1_x, *d_v1_y, *d_v2_x, *d_v2_y;
        bool *d_output;
        int size_float_arr = num*sizeof(float);
        int size_bool_arr = num*sizeof(bool);

        // Alloc space for device copies of vectors
        gpuErrchk  ( cudaMalloc((void **)&d_p_x, sizeof(float)));//Only a single number, not an array
        gpuErrchk  ( cudaMalloc((void **)&d_p_y, sizeof(float)));
        gpuErrchk  ( cudaMalloc((void **)&d_v0_x, size_float_arr));
        gpuErrchk  ( cudaMalloc((void **)&d_v0_y, size_float_arr));
        gpuErrchk  ( cudaMalloc((void **)&d_v1_x, size_float_arr));
        gpuErrchk  ( cudaMalloc((void **)&d_v1_y, size_float_arr));
        gpuErrchk  ( cudaMalloc((void **)&d_v2_x, size_float_arr));
        gpuErrchk  ( cudaMalloc((void **)&d_v2_y, size_float_arr));
        gpuErrchk  ( cudaMalloc((void **)&d_output, size_bool_arr));

        // Copy inputs to device
        gpuErrchk  ( cudaMemcpy(d_p_x, &p_x, sizeof(float), cudaMemcpyHostToDevice));
        gpuErrchk  ( cudaMemcpy(d_p_y, &p_y, sizeof(float), cudaMemcpyHostToDevice));
        gpuErrchk  ( cudaMemcpy(d_v0_x, v0_x, size_float_arr, cudaMemcpyHostToDevice));
        gpuErrchk  ( cudaMemcpy(d_v0_y, v0_y, size_float_arr, cudaMemcpyHostToDevice));
        gpuErrchk  ( cudaMemcpy(d_v1_x, v1_x, size_float_arr, cudaMemcpyHostToDevice));
        gpuErrchk  ( cudaMemcpy(d_v1_y, v1_y, size_float_arr, cudaMemcpyHostToDevice));
        gpuErrchk  ( cudaMemcpy(d_v2_x, v2_x, size_float_arr, cudaMemcpyHostToDevice));
        gpuErrchk  ( cudaMemcpy(d_v2_y, v2_y, size_float_arr, cudaMemcpyHostToDevice));

        // Launch kernel on GPU
        //TO-DO: Determine optimum value for launch parameters
        within_triangle_query<<<num, THREADS_PER_BLOCK>>>
            (d_p_x,
            d_p_y,
            d_v0_x, 
            d_v0_y,
            d_v1_x, 
            d_v1_y, 
            d_v2_x, 
            d_v2_y, 
            d_output);

        gpuErrchk( cudaPeekAtLastError() );
        // Wait for the GPU to finish
        gpuErrchk  ( cudaDeviceSynchronize());
        // Copy result back to host
        gpuErrchk  ( cudaMemcpy(output, d_output, size_bool_arr, cudaMemcpyDeviceToHost));

        // Cleanup
        cudaFree(d_p_x);
        cudaFree(d_p_y);
        cudaFree(d_v0_x );
        cudaFree(d_v0_y);
        cudaFree(d_v1_x );
        cudaFree(d_v1_y );
        cudaFree(d_v2_x );
        cudaFree(d_v2_y );
        cudaFree(d_output);
    }
    else
    {
        //CPU-Version only
        within_triangle_query_cpu(
            p_x,
            p_y,
            v0_x, 
            v0_y,
            v1_x, 
            v1_y, 
            v2_x, 
            v2_y, 
            output,
            num);
    }
    
    return 0;
}

/**
 * Writes data to both machine RAM and GPU RAM as c-style arrays. Sets size variables according to the input data
 * \param tri_v Vector of mesh triangles
 * \param test_points Vector of points to be tested for occlusion
 * \param view_points Camera positions to be tested
 * \param use_gpu Specify whether to execute calculations on the GPU or to fall back to CPU execution 
 */
void setGeometryData_interface(std::vector<TriangleVertices*> &tri_v, std::vector<CartesianCoordinates*> &test_points, std::vector<CartesianCoordinates*> view_points, bool use_gpu)
{
    /*Mesh------------------*/
    g_use_gpu = use_gpu;
    g_mesh_size = tri_v.size();
    g_vertices_size = test_points.size();

    GRID_SIZE = (g_mesh_size+THREADS_PER_BLOCK-1)/THREADS_PER_BLOCK;
    KERNEL_COUNT = GRID_SIZE*THREADS_PER_BLOCK;

    if(GRID_SIZE == 0 || THREADS_PER_BLOCK == 0)
    {
        throw std::runtime_error("CUDA launch parameters cannot be zero");
    }

    h_vertex0 = (float3*)malloc(KERNEL_COUNT*sizeof(float3));
    h_vertex1 = (float3*)malloc(KERNEL_COUNT*sizeof(float3));
    h_vertex2 = (float3*)malloc(KERNEL_COUNT*sizeof(float3));
    h_test_points = (float3*)malloc(g_vertices_size*sizeof(float3));
    h_outputs = (bool*)malloc(g_vertices_size*sizeof(bool));


    //Fill c array with mesh data passed in
    for(int i=0; i<g_mesh_size; i++)
    {
        h_vertex0[i] = make_float3(tri_v[i]->vertices[0]->x, tri_v[i]->vertices[0]->y, tri_v[i]->vertices[0]->z);
        h_vertex1[i] = make_float3(tri_v[i]->vertices[1]->x, tri_v[i]->vertices[1]->y, tri_v[i]->vertices[1]->z);
        h_vertex2[i] = make_float3(tri_v[i]->vertices[2]->x, tri_v[i]->vertices[2]->y, tri_v[i]->vertices[2]->z);
    }
    //Fill remaining array values with zeroes
    for (int i=g_mesh_size; i<KERNEL_COUNT; i++)
    {
        h_vertex0[i] = make_float3(0,0,0);
        h_vertex1[i] = make_float3(0,0,0);
        h_vertex2[i] = make_float3(0,0,0);
    }
    for(int i=0; i<g_vertices_size; i++)
    {
        h_test_points[i] = make_float3(test_points[i]->x, test_points[i]->y, test_points[i]->z);
        h_outputs[i] = true;
    }

    //Allocate space on GPU
    gpuErrchk( cudaMalloc((void **)&d_vertex0, KERNEL_COUNT*sizeof(float3)));
    gpuErrchk( cudaMalloc((void **)&d_vertex1, KERNEL_COUNT*sizeof(float3)));
    gpuErrchk( cudaMalloc((void **)&d_vertex2, KERNEL_COUNT*sizeof(float3)));
    gpuErrchk( cudaMalloc((void **)&d_test_points, g_vertices_size*sizeof(float3)));
    gpuErrchk( cudaMalloc((void **)&d_outputs, g_vertices_size*sizeof(bool)));

    //Copy to device
    gpuErrchk  ( cudaMemcpy(d_vertex0, h_vertex0, KERNEL_COUNT*sizeof(float3), cudaMemcpyHostToDevice));
    gpuErrchk  ( cudaMemcpy(d_vertex1, h_vertex1, KERNEL_COUNT*sizeof(float3), cudaMemcpyHostToDevice));
    gpuErrchk  ( cudaMemcpy(d_vertex2, h_vertex2, KERNEL_COUNT*sizeof(float3), cudaMemcpyHostToDevice));
    gpuErrchk  ( cudaMemcpy(d_test_points, h_test_points, g_vertices_size*sizeof(float3), cudaMemcpyHostToDevice));
    gpuErrchk  ( cudaMemcpy(d_outputs, h_outputs, g_vertices_size*sizeof(bool), cudaMemcpyHostToDevice));

    /*View-Points--------------*/
    g_vp_size = view_points.size();

    h_viewpoints = (float3*)malloc(g_vp_size*sizeof(float3));

    for (int i=0; i<g_vp_size; i++)
    {
        h_viewpoints[i] = make_float3(view_points[i]->x, view_points[i]->y, view_points[i]->z);
    }

    gpuErrchk( cudaMalloc((void **)&d_viewpoints, g_vp_size*sizeof(float3)));

    gpuErrchk  ( cudaMemcpy(d_viewpoints, h_viewpoints, g_vp_size*sizeof(float3), cudaMemcpyHostToDevice));

    g_is_initialized = true;
}

/**
 * Resets the array on the GPU that contains the occlusion results for each test point to all visible.
 * Use after having processed an occlusion query for a given view point and saved its results in order to prepare
 * for a successive query
 */
inline void resetDeviceOutput()
{
    cudaMemset(d_outputs, true, g_vertices_size);
}

/**
 * Decallocate memory previously allocated by setGeometryData_interface()
 */
void deleteGeometryData_interface()
{
    free(h_vertex0);
    free(h_vertex1);
    free(h_vertex2);
    free(h_test_points);
    free(h_viewpoints);
    free(h_outputs);
    
    h_vertex0 = NULL;
    h_vertex1 = NULL;
    h_vertex2 = NULL;
    h_test_points = NULL;
    h_viewpoints = NULL;
    h_outputs = NULL;

    cudaFree(d_vertex0);
    cudaFree(d_vertex1);
    cudaFree(d_vertex2);
    cudaFree(d_test_points);
    cudaFree(d_viewpoints);
    cudaFree(d_outputs);

    d_vertex0 = NULL;
    d_vertex1 = NULL;
    d_vertex2 = NULL;
    d_test_points = NULL;
    d_viewpoints = NULL;
    d_outputs = NULL;

    free(h_viewpoints);
    h_viewpoints = NULL;

    cudaFree(d_viewpoints);
    d_viewpoints = NULL;

    gpuErrchk(cudaDeviceReset());

    g_is_initialized = false;
}

/**
 * CUDA-Kernel for parallelized Moeller-Trumbore ray triangle intersection test. The kernel tests all 
 * mesh triangles for occlusion of a single given test point from a given ray origin.
 * \param ray_origin Pointer to single starting point of the ray for the specific kernel launch
 * \param vertex0_1_2 Pointer to array of vertices of all potential ocludee triangles
 * \param test_point Pointer to the point tested in the specific kernel launch
 * \param output Content of the pointer is set to false if one occlusion is found, left unchanged otherwise
 */
__global__ void occlusionCheck_gpu(float3 *ray_origin, float3 *vertex0, float3 *vertex1, float3 *vertex2, float3 *test_point, bool *output)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;

    float3 ray_vector = *test_point - *ray_origin;
    ray_vector = (1.0/(norm(ray_vector)))*ray_vector; //Normalize

    float3 edge1, edge2, pvec, tvec, qvec;
    float determinant,inv_det,u,v;

    // find vectors for two edges sharing vertex0
    edge1 = vertex1[i] - vertex0[i];
    edge2 = vertex2[i] - vertex0[i];

    //begin calculating determinant - also used to calculate U parameter
    pvec = cross(ray_vector, edge2);

    determinant = dot(edge1,pvec);
    //TO-DO: Consider including incidence angle check here
    //TO-DO: Consider including back face culling from original paper
    //If determinant is near zero, ray lies in plane of triangle
    if (determinant > -EPSILON && determinant < EPSILON)
    {
        return;
    }

    inv_det = 1.0/determinant;
    tvec = *ray_origin - vertex0[i];
    u = inv_det * dot(tvec,pvec);

    //If any single barycentric is sufficiently close to 1, the respective point itself is tested. If so, skip the triangle in question.
    //If a barycentric is exactly 0, the ray may be tangent to the line connecting the other two barycentric points. If so, continue with checks.
    if (u < 0.0 || u >= 1.0-TOLERANCE)
    {
        return;
    }

    qvec  = cross(tvec, edge1);
    v = inv_det*dot(ray_vector,qvec);
    
    if (v < 0.0 || v >= 1.0-TOLERANCE)
    {
        return;
    }

    //Neccessary because boundary intersection is supposed to pass the test (same vertex for multiple triangles) unlike in original algorithm
    //TO-DO: Check with prior statements
    float w = 1 - u - v;
    if(w <= 0.0 || w >= 1.0-TOLERANCE)
    {
        return;
    }
    double t = inv_det * dot(edge2,qvec);
    if (t > EPSILON) // ray intersects triangle
    {
        //Check whether the point is in front of the triangle
        float distp = norm(*test_point-*ray_origin);
        if(t <= distp)
        {
            *output = false;
        }
    }
}

/**
 * CPU implementation of Moeller Trumbore Ray Triangle Intersection Algorithm
 * \param vp_number Index of the vector of view points passed during previous initialization that is to be checked
 * \returns Boolean vector that states whether the entries vertices vector previously passed are visible
 */
std::vector<int> occlusionCheck_cpu(int vp_number)
{
    float3 ray_origin = h_viewpoints[vp_number];

    std::vector<int> occlusion_res(g_vertices_size, true);
    
    //Looping over all vertices 
    for(int vertex_counter=0; vertex_counter<g_vertices_size; vertex_counter++)
    {
        float3 test_point = h_test_points[vertex_counter];
        bool is_visible = true;
        //Looping over all triangles to see if the vertex under test is occluded
        for(int tri_counter=0; tri_counter<g_mesh_size; tri_counter++)
        {
            float3 vertex0 = h_vertex0[tri_counter];
            float3 vertex1 = h_vertex1[tri_counter];
            float3 vertex2 = h_vertex2[tri_counter];

            float3 ray_vector = test_point - ray_origin;
            ray_vector = (1.0/(norm(ray_vector)))*ray_vector; //Normalize

            float3 edge1, edge2, pvec, tvec, qvec;
            float determinant,inv_det,u,v;
            edge1 = vertex1 - vertex0;
            edge2 = vertex2 - vertex0;

            pvec = cross(ray_vector, edge2);
            determinant = dot(edge1,pvec);
            //TO-DO: Consider including incidence angle check here
            //If determinant is near zero, ray lies in plane of triangle
            if (determinant > -EPSILON && determinant < EPSILON)
            {
                continue;
            }

            inv_det = 1.0/determinant;
            tvec = ray_origin - vertex0;
            u = inv_det * dot(tvec,pvec);

            //If any single barycentric is sufficiently close to 1, the respective point itself is tested. If so, skip the triangle in question.
            //If a barycentric is exactly 0, the ray may be tangent to the line connecting the other two barycentric points. If so, continue with checks.
            if (u < 0.0 || u >= 1.0-TOLERANCE)
            {
                continue;
            }

            qvec  = cross(tvec, edge1);
            v = inv_det*dot(ray_vector,qvec);
            
            if (v < 0.0 || v >= 1.0-TOLERANCE)
            {
                continue;
            }

            //TO-DO: Check with prior statements
            float w = 1 - u - v;
            if(w <= 0.0 || w >= 1.0-TOLERANCE)
            {
                continue;
            }

            double t = inv_det * dot(edge2,qvec);
            if (t > EPSILON) // ray intersection
            {
                //Check whether the point is in front of the triangle
                float distp = norm(test_point-ray_origin);
                if(t <= distp)
                {
                    is_visible = false;
                    break;
                }
            }
        }
        occlusion_res[vertex_counter] = is_visible;
    }
    return occlusion_res;
}

/**
 * Performs an occlusion check from the perspective specified the given view point number using Moeller-Trumbore ray triangle intersection test 
 * (Moeller, Trumbore: "Fast, minimum storage ray-triangle intersection.", Journal of Graphics Tools, 2(1):21--28, 1997. )
 * \param vp_number Index of the vector of view points passed during previous initialization that is to be checked
 * \returns Boolean vector that states whether the entries vertices vector previously passed are visible
 */
std::vector<int> occlusionCheck_interface(int vp_number)
{
    if(!g_is_initialized)
    {
        throw std::runtime_error("No geometry data. Initialization routine needs to be called prior to occlusion check");
    }
    if(vp_number > g_vp_size)
    {
        throw std::runtime_error("vp_number parameter passed exceeds the number view points passed during initialization");
    }

    if(g_use_gpu)
    {
        for(int vertex_counter=0; vertex_counter<g_vertices_size; vertex_counter++)
        {
            occlusionCheck_gpu <<<GRID_SIZE, THREADS_PER_BLOCK>>> (&d_viewpoints[vp_number], 
                                                d_vertex0, 
                                                d_vertex1, 
                                                d_vertex2, 
                                                &d_test_points[vertex_counter], 
                                                &d_outputs[vertex_counter]);

            gpuErrchk( cudaPeekAtLastError() );
        }
        // Wait for the GPU to finish
        gpuErrchk  ( cudaDeviceSynchronize());
        // Copy result back to host
        gpuErrchk  ( cudaMemcpy(h_outputs, d_outputs, g_vertices_size*sizeof(bool), cudaMemcpyDeviceToHost));

        //Reset occlusion results in GPU memory to all true
        resetDeviceOutput();

        std::vector<int> output_vect(h_outputs, h_outputs+g_vertices_size);
        return output_vect;
    }
    else
    {
        return occlusionCheck_cpu(vp_number);
    }
}