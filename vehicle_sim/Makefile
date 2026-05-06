# Makefile for Vehicle Performance Simulator
# EE 5103 Final Project
#
# Usage:
#   make          - build the vehicle_sim executable
#   make run      - build and run
#   make clean    - remove build artifacts

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O2 -Iinclude
LDFLAGS  :=

SRC_DIR  := src
OBJ_DIR  := obj
BIN      := vehicle_sim

SOURCES  := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS  := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SOURCES))
DEPS     := $(OBJECTS:.o=.d)

.PHONY: all run clean

all: $(BIN)

$(BIN): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

run: $(BIN)
	./$(BIN)

clean:
	rm -rf $(OBJ_DIR) $(BIN) *.csv

-include $(DEPS)
