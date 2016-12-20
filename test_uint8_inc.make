SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
NV_ARCH=-gencode arch=compute_52,code=compute_52

vpath %.cpp $(SRC_DIR):\
	    $(SRC_DIR)/Tests

vpath %.cu  $(SRC_DIR)/Utilities:\
		    $(SRC_DIR)/Tests

NVCC=/usr/local/cuda/bin/nvcc


# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-c -ccbin=/usr/bin/gcc -std=c++11 -g
LDFLAGS=$(NV_ARCH)
SOURCES =	

CUDA_SOURCES = test_uint8_histo.cu\
			cuda_utilities.cu

# Make a copy wihtou sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
_CUDA_OBJECTS=$(CUDA_SOURCES:.cu=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS)) $(patsubst %,$(OBJ_DIR)/%,$(_CUDA_OBJECTS))


EXECUTABLE=bin/test_uint8_histo
Debug: all

all: $(SOURCES) $(CUDA_SOURCES) $(EXECUTABLE)

$(EXECUTABLE) : $(OBJECTS)
	$(NVCC) $(LDFLAGS) $(OBJECTS) -o $(EXECUTABLE)

$(OBJ_DIR)/%.o : %.cpp
	$(NVCC) $(CFLAGS) $< $(NV_ARCH) -o $(OBJ_DIR)/$(@F)

$(OBJ_DIR)/%.o : %.cu
	$(NVCC) $(CFLAGS) -lineinfo -G -dc $< $(NV_ARCH) -o $(OBJ_DIR)/$(@F)

clean:
	rm $(OBJ_DIR)/*.o $(EXECUTABLE)

run: all
	$(EXECUTABLE)

