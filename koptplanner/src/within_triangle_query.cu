#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include "device_launch_parameters.h"

#define det(u_x, u_y, v_x, v_y) (u_x*v_y-u_y*v_x)


#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
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
int *within_triangle_query_gpu_driver(int num, int threads ,
    float p_x,
    float p_y,
    float *v0_x, 
    float *v0_y,
    float *v1_x, 
    float *v1_y, 
    float *v2_x, 
    float *v2_y, 
    bool *output,
    bool use_gpu) 
{
    if(use_gpu)
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
        //TO-DO: Determine optimum value for number of blocks
        within_triangle_query<<<126, threads>>>
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

        // //Check against CPU version
        // bool *output_cpu;
        // output_cpu = (bool*)malloc(num*sizeof(bool));
        // within_triangle_query_cpu(
        //     p_x,
        //     p_y,
        //     v0_x, 
        //     v0_y,
        //     v1_x, 
        //     v1_y, 
        //     v2_x, 
        //     v2_y, 
        //     output_cpu,
        //     num);

        // for(int i=0; i<num; i++)
        // {
        //     if(output[i] != output_cpu[i])
        //     {
        //         ROS_INFO("False GPU-Result at number %i", i);
        //     }
        // }
        // free(output_cpu);
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
