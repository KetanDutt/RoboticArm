# Native checks for the RoboticArm firmware.
#
#   make -C tests run           build and run the unit tests for common/
#   make -C tests sketch-check  compile AND link both sketches against the
#                               host stubs in arduino_stub/
#   make -C tests check         both of the above (this is what CI runs)
#   make -C tests clean         remove build output
#
# The unit tests compile the exact same .cpp files the Arduino builds use, so
# they can never drift away from the firmware they protect.  sketch-check is
# not a substitute for a real avr-gcc build (see tools/build.sh and CI for
# that) -- it catches typos, bad types, missing definitions and dead
# configuration branches before you ever reach for a board.

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -Wshadow -Wconversion -O1 -g
COMMON   := ../common
INCLUDES := -I$(COMMON) -I.
BIN      := glove_tests
OBJDIR   := build

vpath %.cpp $(COMMON) .

SRC  := $(wildcard $(COMMON)/*.cpp) $(wildcard test_*.cpp) main.cpp
OBJS := $(patsubst %.cpp,$(OBJDIR)/%.o,$(notdir $(SRC)))
DEPS := $(OBJS:.o=.d)

# --- sketch checking ------------------------------------------------------
SKETCHES     := ../hand_transmit ../hand_receive
STUB_DIR     := arduino_stub
SKETCH_BUILD := sketchbuild
# -D__AVR__ so the watchdog branch of each sketch is compiled too.  The strict
# warning set used for the tests is relaxed here because the Arduino core API
# itself is not -Wconversion clean.
# EXTRA_FLAGS lets tools/check_configs.sh build the same sketches with a
# different GloveConfig.h variant (other board profile, other radio driver,
# failsafe disabled, IMU absent, ...) so that every #if branch in the
# configuration is compiled by something.
SKETCH_FLAGS := -std=c++11 -Wall -Wextra -D__AVR__ -I$(STUB_DIR) -I$(COMMON) $(EXTRA_FLAGS)

.PHONY: all run test check sketch-check clean
all: test sketch-check

test run: $(BIN)
	@./$(BIN)

check: test sketch-check
	@echo "ALL HOST CHECKS PASSED"

sketch-check:
	@set -e; \
	for sketch in $(SKETCHES); do \
	  name=$$(basename $$sketch); \
	  echo "== sketch-check: $$name =="; \
	  rm -rf $(SKETCH_BUILD)/$$name; \
	  mkdir -p $(SKETCH_BUILD)/$$name; \
	  cp $$sketch/$$name.ino $(SKETCH_BUILD)/$$name/$$name.cpp; \
	  $(CXX) $(SKETCH_FLAGS) -I$$sketch -c $(SKETCH_BUILD)/$$name/$$name.cpp \
	      -o $(SKETCH_BUILD)/$$name/sketch.o; \
	  for src in $$sketch/*.cpp; do \
	    [ -e "$$src" ] || continue; \
	    $(CXX) $(SKETCH_FLAGS) -I$$sketch -c $$src \
	        -o $(SKETCH_BUILD)/$$name/$$(basename $$src .cpp).o; \
	  done; \
	  $(CXX) $(SKETCH_FLAGS) -c $(STUB_DIR)/stubs.cpp \
	      -o $(SKETCH_BUILD)/$$name/stubs.o; \
	  $(CXX) $(SKETCH_FLAGS) -c $(STUB_DIR)/stub_main.cpp \
	      -o $(SKETCH_BUILD)/$$name/stub_main.o; \
	  $(CXX) $(SKETCH_BUILD)/$$name/*.o -o $(SKETCH_BUILD)/$$name/$$name.elf; \
	  $(SKETCH_BUILD)/$$name/$$name.elf; \
	  echo "   compiled, linked and smoke-ran setup() + 20000 loop() passes"; \
	done

$(BIN): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@

$(OBJDIR)/%.o: %.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

$(OBJDIR):
	@mkdir -p $(OBJDIR)

clean:
	rm -rf $(OBJDIR) $(SKETCH_BUILD) $(BIN)

-include $(DEPS)
