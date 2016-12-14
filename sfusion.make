SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

NV_ARCH=-gencode arch=compute_52,code=compute_52


vpath %.cpp $(SRC_DIR):$(SRC_DIR)/Utilities:\
	$(SRC_DIR)/DataLoader:\
	$(SRC_DIR)/SceneFlowAlgorithm:\
	$(SRC_DIR)/RGBDDevice:\
	$(SRC_DIR)/SceneFusion:\
	third_party/TinyXml:\
	$(SRC_DIR)/Tools:\
	$(SRC_DIR)/TSDF:\
	$(SRC_DIR)/MarchingCubes:\
	$(SRC_DIR)/RayCaster
vpath %.cu  $(SRC_DIR)/GPU:\
			$(SRC_DIR)/MarchingCubes:\
			$(SRC_DIR)/RayCaster:\
			$(SRC_DIR)/SceneFusion:\
			$(SRC_DIR)/TSDF:\
			$(SRC_DIR)/SceneFlowUpdater\
			$(SRC_DIR)/Utilities

NVCC=/usr/local/cuda/bin/nvcc


# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-isystem=/usr/include/eigen3 -I=src/include -I=third_party/TinyXml -ccbin=/usr/bin/gcc -std=c++11 -g
LDFLAGS=$(NV_ARCH) -lpng

SOURCES = tinyxml.cpp tinyxmlparser.cpp tinystr.cpp tinyxmlerror.cpp \
		  FileUtilities.cpp Definitions.cpp\
		  MockSceneFlowAlgorithm.cpp \
		  SRSFMockSceneFlowAlgorithm.cpp \
		  PDSFMockSceneFlowAlgorithm.cpp \
		  MockKinect.cpp \
		  Camera.cpp \
		  DepthImage.cpp \
          PngUtilities.cpp PngWrapper.cpp RenderUtilities.cpp \
			SceneFusion.cpp\
          sfusion.cpp ply.cpp 


CUDA_SOURCES = 	MarkAndSweepMC.cu\
				cuda_utilities.cu\
				TSDFVolume.cu\
				TSDF_utilities.cu\
				GPURaycaster.cu\
				SceneFusion_krnl.cu

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
	$(NVCC) -c $(CFLAGS) $< $(NV_ARCH) -o $(OBJ_DIR)/$(@F)

$(OBJ_DIR)/%.o : %.cu
	$(NVCC) -c -G -g $(CFLAGS) -lineinfo -dc $< $(NV_ARCH) -o $(OBJ_DIR)/$(@F)

clean:
	rm $(OBJ_DIR)/*.o $(EXECUTABLE)

run: all
	$(EXECUTABLE)

