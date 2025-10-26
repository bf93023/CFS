# Makefile - portable C++ build (works with g++ / clang++)
# Usage examples:
#   make                # release build with default compiler (g++)
#   make BUILD=debug    # debug build
#   make CXX=clang++    # override compiler
#   make clean
#   make run

# ---- Configurable variables (override on command line) ----
CXX ?= g++
BUILD ?= release            # set to "debug" for debug builds
SRC_DIR := src
OBJ_DIR := obj
OUT := main

# Include directories
INCLUDES := -I include -I entt

# Common flags
CPPSTD := -std=c++17
DEPFLAGS := -MMD -MP         # auto-dependency generation

ifeq ($(BUILD),debug)
    CXXFLAGS := $(CPPSTD) -g -O0 -Wall -Wextra
else
    CXXFLAGS := $(CPPSTD) -O3 -Wall -Wextra -march=native
endif

CPPFLAGS := $(INCLUDES) $(DEPFLAGS)

# Collect sources and compute object names
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)

# ---- Targets ----
.PHONY: all clean run dist dirs

all: dirs $(OUT)

# Link
$(OUT): $(OBJS)
	@echo "Linking -> $@"
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile rule with dependency generation
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@echo "Compiling $<"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

# Ensure object dir exists
dirs:
	@mkdir -p $(OBJ_DIR)

# Run the program (only if built)
run: all
	@./$(OUT)

# Clean build artifacts
clean:
	@echo "Cleaning..."
	@rm -rf $(OBJ_DIR) $(OUT) $(DEPS)

# Package binary + resources (optional)
dist: all
	@mkdir -p dist
	@cp $(OUT) dist/
	@echo "Packaged to dist/"

# Include auto-generated dependency files if present
-include $(DEPS)

