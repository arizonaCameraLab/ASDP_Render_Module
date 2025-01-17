#include <cuda_runtime.h>
#include <iostream>
#include <chrono>

#define NX 1280
#define NY 1024
#define COUNT 1000

__global__ void sumFloatKernelAtomic(const uint16_t* image, float* sum, float* sumOfSquares, int width, int height) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  int index = idy * width + idx;

  if (idx < width && idy < height) {
    atomicAdd(sum, static_cast<float>(image[index]));
    atomicAdd(sumOfSquares, static_cast<float>(image[index]) * static_cast<float>(image[index]));
  }
}

__global__ void sumFloatKernelReduce(const uint16_t* image, float* sum, float* sumOfSquares, int width, int height) {
  __shared__ float sharedSum[256];
  __shared__ float sharedSumOfSquares[256];

  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  int index = idy * width + idx;
  int tid = threadIdx.y * blockDim.x + threadIdx.x;

  float pixelValue = 0.0f;
  float pixelValueSquared = 0.0f;

  if (idx < width && idy < height) {
    pixelValue = static_cast<float>(image[index]);
    pixelValueSquared = pixelValue * pixelValue;
  }

  sharedSum[tid] = pixelValue;
  sharedSumOfSquares[tid] = pixelValueSquared;

  __syncthreads();

  // Reduce within block
  for (int stride = blockDim.x * blockDim.y / 2; stride > 0; stride >>= 1) {
    if (tid < stride) {
      sharedSum[tid] += sharedSum[tid + stride];
      sharedSumOfSquares[tid] += sharedSumOfSquares[tid + stride];
    }
    __syncthreads();
  }

  // One thread per block writes the result to global memory
  if (tid == 0) {
    atomicAdd(sum, sharedSum[0]);
    atomicAdd(sumOfSquares, sharedSumOfSquares[0]);
  }
}

__global__ void sumUint64KernelAtomic(const uint16_t* image, uint64_t* sum, uint64_t* sumOfSquares, int width, int height) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  int index = idy * width + idx;

  if (idx < width && idy < height) {
    atomicAdd(sum, static_cast<uint64_t>(image[index]));
    atomicAdd(sumOfSquares, static_cast<uint64_t>(image[index]) * static_cast<uint64_t>(image[index]));
  }
}

__global__ void sumUint64KernelReduce(const uint16_t* image, uint64_t* sum, uint64_t* sumOfSquares, int width, int height) {
  __shared__ uint64_t sharedSum[256];
  __shared__ uint64_t sharedSumOfSquares[256];

  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  int index = idy * width + idx;
  int tid = threadIdx.y * blockDim.x + threadIdx.x;

  uint64_t pixelValue = 0;
  uint64_t pixelValueSquared = 0;

  if (idx < width && idy < height) {
    pixelValue = static_cast<uint64_t>(image[index]);
    pixelValueSquared = pixelValue * pixelValue;
  }

  sharedSum[tid] = pixelValue;
  sharedSumOfSquares[tid] = pixelValueSquared;

  __syncthreads();

  // Reduce within block
  for (int stride = blockDim.x * blockDim.y / 2; stride > 0; stride >>= 1) {
    if (tid < stride) {
      sharedSum[tid] += sharedSum[tid + stride];
      sharedSumOfSquares[tid] += sharedSumOfSquares[tid + stride];
    }
    __syncthreads();
  }

  // One thread per block writes the result to global memory
  if (tid == 0) {
    atomicAdd(sum, sharedSum[0]);
    atomicAdd(sumOfSquares, sharedSumOfSquares[0]);
  }
}

void benchmarkSumFloat(const uint16_t* d_image, int width, int height, int count) {
  float* d_sum, * d_sumOfSquares;
  cudaMalloc(&d_sum, sizeof(float));
  cudaMalloc(&d_sumOfSquares, sizeof(float));
  cudaMemset(d_sum, 0, sizeof(float));
  cudaMemset(d_sumOfSquares, 0, sizeof(float));

  dim3 blockSize(16, 16);
  dim3 gridSize((width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y);

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < count; ++i) {
    sumFloatKernelAtomic << <gridSize, blockSize >> > (d_image, d_sum, d_sumOfSquares, width, height);
    cudaDeviceSynchronize();
  }
  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> duration = end - start;
  std::cout << "Float32 sum atomic duration: " << duration.count() / count * 1e3 << " milliseconds" << std::endl;

  start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < count; ++i) {
    sumFloatKernelReduce << <gridSize, blockSize >> > (d_image, d_sum, d_sumOfSquares, width, height);
    cudaDeviceSynchronize();
  }
  end = std::chrono::high_resolution_clock::now();

  duration = end - start;
  std::cout << "Float32 sum reduce duration: " << duration.count() / count * 1e3 << " milliseconds" << std::endl;

  cudaFree(d_sum);
  cudaFree(d_sumOfSquares);
}

void benchmarkSumUint64(const uint16_t* d_image, int width, int height, int count) {
  uint64_t* d_sum, * d_sumOfSquares;
  cudaMalloc(&d_sum, sizeof(uint64_t));
  cudaMalloc(&d_sumOfSquares, sizeof(uint64_t));
  cudaMemset(d_sum, 0, sizeof(uint64_t));
  cudaMemset(d_sumOfSquares, 0, sizeof(uint64_t));

  dim3 blockSize(16, 16);
  dim3 gridSize((width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y);

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < count; ++i) {
    sumUint64KernelAtomic << <gridSize, blockSize >> > (d_image, d_sum, d_sumOfSquares, width, height);
    cudaDeviceSynchronize();
  }
  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> duration = end - start;
  std::cout << "Uint64 sum atomic duration: " << duration.count() / count * 1e3 << " milliseconds" << std::endl;

  start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < count; ++i) {
    sumUint64KernelReduce << <gridSize, blockSize >> > (d_image, d_sum, d_sumOfSquares, width, height);
    cudaDeviceSynchronize();
  }
  end = std::chrono::high_resolution_clock::now();

  duration = end - start;
  std::cout << "Uint64 sum reduce duration: " << duration.count() / count * 1e3 << " milliseconds" << std::endl;

  cudaFree(d_sum);
  cudaFree(d_sumOfSquares);
}

int main() {
  const int size = NX * NY;
  uint16_t* h_image = new uint16_t[size];
  for (int i = 0; i < size; ++i) {
    h_image[i] = static_cast<uint16_t>(i % 256);
  }

  uint16_t* d_image;
  cudaMalloc(&d_image, size * sizeof(uint16_t));
  cudaMemcpy(d_image, h_image, size * sizeof(uint16_t), cudaMemcpyHostToDevice);

  benchmarkSumFloat(d_image, NX, NY, COUNT);
  benchmarkSumUint64(d_image, NX, NY, COUNT);

  cudaFree(d_image);
  delete[] h_image;

  return 0;
}
