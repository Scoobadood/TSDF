vpath %.cpp KinFu:KinFu/Utilities

SRC_DIR = KinFu
OBJ_DIR = obj
CC=/usr/bin/g++

# use isystem for eigen as it forces compiler to supress warnings from
# those files. Eigen generates a lot
CFLAGS=-I/usr/local/include/eigen3 -std=c++11 -g -c
LDFLAGS=-lpng

SOURCES = PgmUtilities.cpp PngUtilities.cpp DepthMapUtilities.cpp pgm2png.cpp 


# Make a copy wihtou sub directories
_OBJECTS=$(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJECTS))


EXECUTABLE=pgm2png
Debug: all

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE) : $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $(EXECUTABLE)

$(OBJ_DIR)/%.o : %.cpp
	$(CC) $(CFLAGS) $< -o $(OBJ_DIR)/$(@F)

clean:
	rm $(OBJ_DIR)/*.o $(EXECUTABLE)

run: all
	$(EXECUTABLE)

