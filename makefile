CXX      := -g++
CXXFLAGS := -pedantic-errors -Wall -Wextra -Werror
LDFLAGS  := -L/usr/lib -lstdc++ -lm
BUILD    := ./build
OBJ_DIR  := $(BUILD)/objects
TARGET   := main
INCLUDE  := -Iinclude/

LIB_DIRS := $(wildcard lib/*/) 
LIBS     := $(LIB_DIRS:%=-I%)

SRC      :=                      \
   $(wildcard src/*/*.cpp) 		 \
   $(wildcard src/*.cpp)         \
   $(wildcard include/*/*.cpp)	 \
   $(wildcard include/*.cpp)	 \
   $(wildcard lib/*/*.cpp)

OBJECTS  := $(SRC:%.cpp=$(OBJ_DIR)/%.o)

all: build $(TARGET)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(LIBS) -c $< -o $@ $(LDFLAGS)

$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $^ $(LDFLAGS)

.PHONY: all build clean debug release

build:
	@mkdir -p $(OBJ_DIR)

debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O2
release: all

clean:
	-@rm -rvf $(OBJ_DIR)/*
