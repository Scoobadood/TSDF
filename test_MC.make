SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
NV_ARCH=-gencode arch=compute_52,code=compute_52
NVCC=/usr/local/cuda/bin/nvcc

# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-c -isystem=/usr/include/eigen3 -ccbin=/usr/bin/gcc -std=c++11 -g
LDFLAGS=$(NV_ARCH) -lpng


vpath %.cpp $(SRC_DIR):\
	    $(SRC_DIR)/Utilities:\
	    $(SRC_DIR)/DataLoader:\
	    $(SRC_DIR)/SceneFlowAlgorithm:\
	    $(SRC_DIR)/SceneFusion:\
	    $(SRC_DIR)/RGBDDevice:\
	    $(SRC_DIR)/Tools:third_party/TinyXml:\
	    $(SRC_DIR)/TSDF:\
	    $(SRC_DIR)/MarchingCubes:\
	    $(SRC_DIR)/RayCaster:\
	    $(SRC_DIR)/Tests

vpath %.cu  $(SRC_DIR)/Utilities:\
			$(SRC_DIR)/MarchingCubes:\
			$(SRC_DIR)/RayCaster:\
			$(SRC_DIR)/TSDF:\
			$(SRC_DIR)/SceneFlowUpdater





SOURCES =	FileUtilities.cpp Definitions.cpp\
			Camera.cpp \
			DepthImage.cpp \
			PngUtilities.cpp PngWrapper.cpp \
			RenderUtilities.cpp PgmUtilities.cpp \
			TUMDataLoader.cpp \
			test_MC_main.cpp ply.cpp 

#SOURCES = BilateralFilter.cpp Camera.cpp \
#          BlockTSDFLoader.cpp GPURaycaster.cpp \
#          Definitions.cpp DepthMapUtilities.cpp FileUtilities.cpp PgmUtilities.cpp \
#          PngUtilities.cpp PngWrapper.cpp RenderUtilities.cpp TSDFLoader.cpp \
#          CPURaycaster.cpp CPUTSDFVolume.cpp \
#          DepthImage.cpp TUMDataLoader.cpp \
#          test_MC_main.cpp


CUDA_SOURCES = TSDFVolume.cu MarkAndSweepMC.cu TSDF_utilities.cu cuda_utilities.cu GPURaycaster.cu cuda_coordinate_transforms.cu


# Make a copy wihtou sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
_CUDA_OBJECTS=$(CUDA_SOURCES:.cu=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS)) $(patsubst %,$(OBJ_DIR)/%,$(_CUDA_OBJECTS))


EXECUTABLE=$(BIN_DIR)/test_MC
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

