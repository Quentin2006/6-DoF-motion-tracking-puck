CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g
LDFLAGS = -lglfw -lGL -ldl -lpthread

SRCDIR = src
INCDIR = include
BINDIR = bin
OBJDIR = obj

CPP_SOURCES = $(shell find $(SRCDIR) -name "*.cpp")
C_SOURCES = $(shell find $(SRCDIR) -name "*.c")
SOURCES = $(CPP_SOURCES) $(C_SOURCES)
CPP_OBJECTS = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(CPP_SOURCES))
C_OBJECTS = $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(C_SOURCES))
OBJECTS = $(CPP_OBJECTS) $(C_OBJECTS)
TARGET = $(BINDIR)/opengl_template

.PHONY: all clean run

all: $(TARGET)

DIRS = $(sort $(dir $(OBJECTS)))

$(TARGET): $(OBJECTS) | $(BINDIR) $(DIRS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

$(DIRS):
	mkdir -p $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -c $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(@D)
	$(CC) -I$(INCDIR) -c $< -o $@

$(BINDIR):
	mkdir -p $(BINDIR)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf $(OBJDIR) $(BINDIR)
