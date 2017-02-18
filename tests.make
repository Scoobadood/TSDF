SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

NV_ARCH=-gencode arch=compute_52,code=compute_52

vpath %.cpp $(SRC_DIR):\
	$(SRC_DIR)/Tests:\
	$(SRC_DIR)/Tests/TestTSDF:\
	$(SRC_DIR)/GPU:\
	$(SRC_DIR)/Utilities:\
	$(SRC_DIR)/Raycaster:\
	$(SRC_DIR)/DataLoader:\
	$(SRC_DIR)/Tools:\
	$(SRC_DIR)/TSDF

vpath %.cu  $(SRC_DIR)/GPU:\
			$(SRC_DIR)/MarchingCubes:\
			$(SRC_DIR)/RayCaster:\
			$(SRC_DIR)/TSDF:\
			$(SRC_DIR)/SceneFlowUpdater\
			$(SRC_DIR)/Utilities

NVCC=/usr/local/cuda/bin/nvcc


# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CC=/usr/bin/gcc
CFLAGS=-isystem /usr/include/eigen3 -I src -isystem=/usr/include -c -std=c++11 -g
NVCFLAGS=$(CFLAGS) -G -ccbin=$(CC) 
LDFLAGS=$(NV_ARCH) -lpng -lgtest

SOURCES = Camera.cpp \
          Definitions.cpp DepthMapUtilities.cpp FileUtilities.cpp PgmUtilities.cpp \
          PngUtilities.cpp PngWrapper.cpp RenderUtilities.cpp \
          DepthImage.cpp TUMDataLoader.cpp ply.cpp \
          TestHelpers.cpp 

CUDA_SOURCES =	TSDFVolume.cu TSDF_utilities.cu\
				MarkAndSweepMC.cu\
				GPURaycaster.cu\
				cuda_utilities.cu\
				cuda_coordinate_transforms.cu

# Make a copy without sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
_CUDA_OBJECTS=$(CUDA_SOURCES:.cu=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS)) $(patsubst %,$(OBJ_DIR)/%,$(_CUDA_OBJECTS))

all: Integration Raycasting Cam TSDF_LoadSave

Integration: $(BIN_DIR)/test_integrate
Raycasting:  $(BIN_DIR)/test_raycast
Cam:  $(BIN_DIR)/test_camera
TSDF_LoadSave:    $(BIN_DIR)/test_tsdf_load_save

	
$(BIN_DIR)/test_camera: $(OBJECTS) $(OBJ_DIR)/Test_Camera.o
	$(NVCC) $(LDFLAGS) $(OBJECTS) $(OBJ_DIR)/Test_Camera.o -o $(BIN_DIR)/test_camera

$(BIN_DIR)/test_integrate: $(OBJECTS) $(OBJ_DIR)/Test_TSDF_Integration.o
	$(NVCC) $(LDFLAGS) $(OBJECTS) $(OBJ_DIR)/Test_TSDF_Integration.o -o $(BIN_DIR)/test_integrate

$(BIN_DIR)/test_raycast: $(OBJECTS) $(OBJ_DIR)/Test_TSDF_RayCast.o 
	$(NVCC) $(LDFLAGS) $(OBJECTS) $(OBJ_DIR)/Test_TSDF_RayCast.o -o $(BIN_DIR)/test_raycast

$(BIN_DIR)/test_tsdf_load_save: $(OBJECTS) $(OBJ_DIR)/Test_TSDF_Load_Save.o
	$(NVCC) $(LDFLAGS) $(OBJECTS) $(OBJ_DIR)/Test_TSDF_Load_Save.o -o $(BIN_DIR)/test_tsdf_load_save

$(OBJ_DIR)/%.o : %.cpp
	$(NVCC) $(NVCFLAGS) $< $(NV_ARCH) -o $(OBJ_DIR)/$(@F)

$(OBJ_DIR)/%.o : %.cu
	$(NVCC) -c -g $(NVCFLAGS) -lineinfo -dc $< $(NV_ARCH) -o $(OBJ_DIR)/$(@F)

clean:
	rm $(OBJ_DIR)/*.o $(EXECUTABLE)

run: all
	$(EXECUTABLE)

