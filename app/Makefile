CXX = g++

PYTHON_VERSION = 3.12
PYTHON_INCLUDE = /mingw64/include/python$(PYTHON_VERSION)
PYTHON_LIB = /mingw64/lib/libpython$(PYTHON_VERSION).dll.a
PYBIND11_INCLUDE = ./pybind11/include
PYTHON_CONFIG = python$(PYTHON_VERSION)-config


EXTERNAL_LIB_PATH = ./ti-external
EXTERNAL_LIBS = -L$(EXTERNAL_LIB_PATH) -lReleaseRF_API


EXTERNAL_INCLUDES = -I$(EXTERNAL_LIB_PATH)/RF_API -I$(PYBIND11_INCLUDE) -I$(PYTHON_INCLUDE)


CXXFLAGS = -std=c++14 -shared -fPIC -static-libgcc -static-libstdc++ $(EXTERNAL_INCLUDES) -DENABLE_DEBUG


LDFLAGS = $(shell $(PYTHON_CONFIG) --ldflags) -L/mingw64/lib -lmingw32 -mwindows $(EXTERNAL_LIBS)


SRC = radar_api.cpp


TARGET = radar_api.pyd


all: $(TARGET)


$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)


clean:
	rm -f $(TARGET)

.PHONY: all clean
