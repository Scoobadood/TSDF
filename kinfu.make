SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

vpath %.cpp $(SRC_DIR):$(SRC_DIR)/Tests:$(SRC_DIR)/GPU:$(SRC_DIR)/CPU:$(SRC_DIR)/Utilities:$(SRC_DIR)/DataLoader:$(SRC_DIR)/Tools
vpath %.cu $(SRC_DIR)/GPU

NVCC=/usr/local/cuda/bin/nvcc


# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-isystem=/usr/include/eigen3 -isystem=/usr/local/include/eigen3 -I=src  -c  -ccbin=/usr/bin/gcc -std=c++11 -g
LDFLAGS=-lpng

SOURCES = kinfu.cpp BilateralFilter.cpp TSDFVolume.cpp Camera.cpp \
          BlockTSDFLoader.cpp GPURaycaster.cpp \
          Definitions.cpp DepthMapUtilities.cpp FileUtilities.cpp PgmUtilities.cpp \
          PngUtilities.cpp PngWrapper.cpp RenderUtilities.cpp TSDFLoader.cpp \
          CPURaycaster.cpp CPUTSDFVolume.cpp \
          DepthImage.cpp TUMDataLoader.cpp ply.cpp 


CUDA_SOURCES = Raycaster_kernel.cu GPUTSDFVolume.cu MarchingCubes.cu TSDF_utilities.cu cu_common.cu

# Make a copy wihtou sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
_CUDA_OBJECTS=$(CUDA_SOURCES:.cu=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS)) $(patsubst %,$(OBJ_DIR)/%,$(_CUDA_OBJECTS))


EXECUTABLE=$(BIN_DIR)/kinfu
Debug: all

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE) : $(OBJECTS)
	$(NVCC) $(LDFLAGS) $(OBJECTS) -o $(EXECUTABLE)

$(OBJ_DIR)/%.o : %.cpp
	$(NVCC) $(CFLAGS) $< -o $(OBJ_DIR)/$(@F)

$(OBJ_DIR)/%.o : %.cu
	$(NVCC) $(CFLAGS) -lineinfo -dc $< -o $(OBJ_DIR)/$(@F)

clean:
	rm $(OBJ_DIR)/*.o $(EXECUTABLE)

run: all
	$(EXECUTABLE)

