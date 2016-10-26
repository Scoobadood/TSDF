SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

vpath %.cpp $(SRC_DIR):$(SRC_DIR)/Utilities:$(SRC_DIR)/DataLoader:$(SRC_DIR)/SceneFlowAlgorithm:$(SRC_DIR)/SceneFusion:$(SRC_DIR)/RGBDDevice:$(SRC_DIR)/Tools:third_party/TinyXml:$(SRC_DIR)/TSDF:$(SRC_DIR)/MarchingCubes:$(SRC_DIR)/RayCaster
vpath %.cu $(SRC_DIR)/GPU:$(SRC_DIR)/MarchingCubes:$(SRC_DIR)/RayCaster:$(SRC_DIR)/TSDF:$(SRC_DIR)/SceneFlowUpdater

NVCC=/usr/local/cuda/bin/nvcc


# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-isystem=/usr/include/eigen3 -isystem=/usr/local/include/eigen3 -I=src/include -I=third_party/TinyXml -ccbin=/usr/bin/gcc -std=c++11 -g
LDFLAGS=-lpng

SOURCES = tinyxml.cpp tinyxmlparser.cpp tinystr.cpp tinyxmlerror.cpp \
		  FileUtilities.cpp Definitions.cpp\
		  SRSFMockSceneFlowAlgorithm.cpp \
		  MockKinect.cpp \
		  TSDFVolume.cpp Camera.cpp \
		  GPURaycaster.cpp \
		  DepthImage.cpp SceneFusion.cpp \
          PngUtilities.cpp PngWrapper.cpp \
          sfusion.cpp


CUDA_SOURCES = GPUMarchingCubes.cu cu_common.cu GPUTSDFVolume.cu TSDF_utilities.cu Raycaster_kernel.cu SceneFlowUpdater.cu

# Make a copy wihtou sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
_CUDA_OBJECTS=$(CUDA_SOURCES:.cu=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS))   $(patsubst %,$(OBJ_DIR)/%,$(_CUDA_OBJECTS))


EXECUTABLE=$(BIN_DIR)/sfusion
Debug: all

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE) : $(OBJECTS)
	$(NVCC) $(LDFLAGS) $(OBJECTS) -o $(EXECUTABLE)

$(OBJ_DIR)/%.o : %.cpp
	$(NVCC) -c $(CFLAGS) $< -o $(OBJ_DIR)/$(@F)

$(OBJ_DIR)/%.o : %.cu
	$(NVCC) -c $(CFLAGS) -lineinfo -dc $< -o $(OBJ_DIR)/$(@F)

clean:
	rm $(OBJ_DIR)/*.o $(EXECUTABLE)

run: all
	$(EXECUTABLE)

