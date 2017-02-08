SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
THIRD_PTY = third_party

NV_VERSION=52

#Add the following line for cluster builds
#GCC_ROOT=/opt/rh/devtoolset-2/root

NV_ARCH=-gencode arch=compute_$(NV_VERSION),code=compute_$(NV_VERSION)
NVCC=/usr/local/cuda/bin/nvcc

# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-c -isystem=/usr/include/eigen3 -I$(THIRD_PTY) -ccbin=$(GCC_ROOT)/usr/bin/gcc -std=c++11 -g
LDFLAGS=$(NV_ARCH) -lpng


vpath %.cpp $(SRC_DIR):$(SRC_DIR)/Utilities:\
	$(SRC_DIR)/DataLoader:\
	$(SRC_DIR)/SceneFlowAlgorithm:\
	$(SRC_DIR)/RGBDDevice:\
	$(SRC_DIR)/SceneFusion:\
	$(SRC_DIR)/Tools:\
	$(SRC_DIR)/TSDF:\
	$(SRC_DIR)/MarchingCubes:\
	$(SRC_DIR)/RayCaster:\
	$(THIRD_PTY)/ICP_CUDA:\
	$(THIRD_PTY)/ICP_CUDA/Cuda/containers

vpath %.cu  $(SRC_DIR)/GPU:\
			$(SRC_DIR)/MarchingCubes:\
			$(SRC_DIR)/RayCaster:\
			$(SRC_DIR)/SceneFusion:\
			$(SRC_DIR)/TSDF:\
			$(SRC_DIR)/SceneFlowUpdater:\
			$(SRC_DIR)/Utilities:\
			$(THIRD_PTY)/ICP_CUDA/Cuda:\
			$(THIRD_PTY)/ICP_CUDA/Cuda/containers


SOURCES = FileUtilities.cpp Definitions.cpp\
		  Camera.cpp \
		  DepthImage.cpp \
          PngUtilities.cpp PngWrapper.cpp RenderUtilities.cpp \
          ICPOdometry.cpp device_memory.cpp initialization.cpp \
          ply.cpp \
          tsdf_icp.cpp


CUDA_SOURCES = 	MarkAndSweepMC.cu\
				cuda_utilities.cu\
				cuda_coordinate_transforms.cu\
				TSDFVolume.cu\
				TSDF_utilities.cu\
				GPURaycaster.cu\
				estimate.cu pyrdown.cu 



# Make a copy wihtou sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
_CUDA_OBJECTS=$(CUDA_SOURCES:.cu=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS)) $(patsubst %,$(OBJ_DIR)/%,$(_CUDA_OBJECTS))


EXECUTABLE=$(BIN_DIR)/tsdf_icp
Debug: all

all: $(SOURCES) $(CUDA_SOURCES) $(EXECUTABLE)

$(EXECUTABLE) : $(OBJECTS)
	$(NVCC) $(LDFLAGS) $(OBJECTS) -o $(EXECUTABLE)

$(OBJ_DIR)/%.o : %.cpp
	$(NVCC) $(CFLAGS)                     $< $(NV_ARCH) -o $(OBJ_DIR)/$(@F)

$(OBJ_DIR)/%.o : %.cu
	$(NVCC) -G -g $(CFLAGS) -lineinfo -dc $< $(NV_ARCH) -o $(OBJ_DIR)/$(@F) 

clean:
	rm $(OBJ_DIR)/*.o $(EXECUTABLE)

run: all
	$(EXECUTABLE)

