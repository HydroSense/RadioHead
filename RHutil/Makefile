# ref: http://hiltmon.com/blog/2013/07/03/a-simple-c-plus-plus-project-structure/
# ref: http://stackoverflow.com/questions/1139271/makefiles-with-source-files-in-different-directories

CC := g++ # This is the main compiler
# CC := clang --analyze # and comment out the linker last line for sanity
SRCDIR := .
BUILDDIR := ../build
# TARGET := bin/DQNserver

SRCEXT := cpp
SOURCES := $(shell find $(SRCDIR) -type f -name '*.$(SRCEXT)')
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
#removed -DBCM2835_NO_DELAY_COMPATIBILITY, not needed?
CFLAGS := -DRASPBERRY_PI -g -lm -Wall
LIB := -lwiringPi
INC := -I. -I..
#
# $(TARGET): $(OBJECTS)
# 	@mkdir -p bin
# 	@echo " Linking..."
# 	@echo " $(CC) $^ -o $(TARGET) $(LIB)"; $(CC) $^ -o $(TARGET) $(LIB)
all: $(OBJECTS)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<

clean:
	@echo " Cleaning...";
	# @echo " $(RM) -r $(BUILDDIR) $(TARGET)"; $(RM) -r $(BUILDDIR) $(TARGET)
	$(RM) -r $(BUILDDIR)

.PHONY: clean
