vpath %.cpp KinFu:KinFu/Tests:KinFu/GPU:KinFu/CPU:KinFu/Utilities:KinFu/DataLoader
vpath %.cu KinFu/GPU

SRC_DIR = KinFu
OBJ_DIR = obj
NVCC=/usr/local/cuda/bin/nvcc


# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-isystem=/usr/include/eigen3 -c  -ccbin=/usr/bin/gcc-4.8 -std=c++11 -g
LDFLAGS=-lpng

SOURCES = BilateralFilter.cpp TSDFVolume.cpp Camera.cpp \
          BlockTSDFLoader.cpp GPURaycaster.cpp \
          Definitions.cpp DepthMapUtilities.cpp FileUtilities.cpp PgmUtilities.cpp \
          PngUtilities.cpp PngWrapper.cpp RenderUtilities.cpp TSDFLoader.cpp \
          CPURaycaster.cpp CPUTSDFVolume.cpp \
          DepthImage.cpp TUMDataLoader.cpp \
          test_MC_main.cpp


CUDA_SOURCES = TSDF_kernel.cu GPUTSDFVolume.cu MarchingCubes.cu TSDF_utilities.cu Raycaster_kernel.cu cu_common.cu

# Make a copy wihtou sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
_CUDA_OBJECTS=$(CUDA_SOURCES:.cu=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS)) $(patsubst %,$(OBJ_DIR)/%,$(_CUDA_OBJECTS))


EXECUTABLE=test_MC
Debug: all

all: $(SOURCES) $(CUDA_SOURCES) $(EXECUTABLE)

$(EXECUTABLE) : $(OBJECTS)
	$(NVCC) $(LDFLAGS) $(OBJECTS) -o $(EXECUTABLE)

$(OBJ_DIR)/%.o : %.cpp
	$(NVCC) $(CFLAGS) $< -o $(OBJ_DIR)/$(@F)

$(OBJ_DIR)/%.o : %.cu
	$(NVCC) $(CFLAGS) -lineinfo -G -dc $< -o $(OBJ_DIR)/$(@F)

clean:
	rm $(OBJ_DIR)/*.o $(EXECUTABLE)

run: all
	$(EXECUTABLE)

