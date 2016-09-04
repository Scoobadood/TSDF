SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

vpath %.cpp $(SRC_DIR):$(SRC_DIR)/Utilities:$(SRC_DIR)/DataLoader:$(SRC_DIR)/SceneFlowAlgorithm:$(SRC_DIR)/SceneFusion:$(SRC_DIR)/RGBDDevice:$(SRC_DIR)/Tools:third_party/TinyXml
vpath %.cu $(SRC_DIR):$(SRC_DIR)/Utilities:$(SRC_DIR)/DataLoader:$(SRC_DIR)/SceneFlowAlgorithm:$(SRC_DIR)/SceneFusion

NVCC=/usr/local/cuda/bin/nvcc


# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-isystem=/usr/include/eigen3 -isystem=/usr/local/include/eigen3 -I=src  -I=third_party/TinyXml -c  -ccbin=/usr/bin/gcc -std=c++11 -g
LDFLAGS=-lpng

SOURCES = tinyxml.cpp tinyxmlparser.cpp tinystr.cpp tinyxmlerror.cpp \
		  FileUtilities.cpp\
		  SRSFMockSceneFlowAlgorithm.cpp \
		  MockKinect.cpp \
		  DepthImage.cpp SceneFusion.cpp \
          PngUtilities.cpp PngWrapper.cpp \
          sfusion.cpp


CUDA_SOURCES = 

# Make a copy wihtou sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
_CUDA_OBJECTS=$(CUDA_SOURCES:.cu=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS)) $(patsubst %,$(OBJ_DIR)/%,$(_CUDA_OBJECTS))


EXECUTABLE=$(BIN_DIR)/sfusion
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

