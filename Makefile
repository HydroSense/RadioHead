# ref: http://hiltmon.com/blog/2013/07/03/a-simple-c-plus-plus-project-structure/
# ref: http://stackoverflow.com/questions/1139271/makefiles-with-source-files-in-different-directories

CC := g++ # This is the main compiler
# CC := clang --analyze # and comment out the linker last line for sanity
SRCDIR := .
BUILDDIR := build
TARGET := bin/libradiohead.a
# TARGET := bin/DQNserver

SRCEXT := cpp
SOURCES := $(shell find $(SRCDIR) -type f -name '*.$(SRCEXT)')
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
#removed -DBCM2835_NO_DELAY_COMPATIBILITY, not needed?
override CFLAGS += -DRASPBERRY_PI -g -lm -Wall
LIB := -lwiringPi
INC := -I. -I./RHutil
#
# $(TARGET): $(OBJECTS)
# 	@mkdir -p bin
# 	@echo " Linking..."
# 	@echo " $(CC) $^ -o $(TARGET) $(LIB)"; $(CC) $^ -o $(TARGET) $(LIB)
$(TARGET): $(OBJECTS)
	@echo " Linking..."
	@mkdir -p $(@D)
	@echo " $(AR) rcs -o $(TARGET) $(OBJECTS)"; $(AR) rcs -o $(TARGET) $(OBJECTS);

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
#@mkdir -p $(BUILDDIR)
	@mkdir -p $(@D)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<

clean:
	@echo " Cleaning...";
	@echo " $(RM) -r $(BUILDDIR) bin"; $(RM) -r $(BUILDDIR) bin

# blink
blink:
	$(CC) $(CFLAGS) test/tester.cpp $(INC) $(LIB) -o bin/tester


.PHONY: clean
