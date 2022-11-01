CC = g++

CFLAGS = -Wall

TARGET = soma_rpi

SRCS = ./src/main.cpp
SRCS += ./src/Clutch.cpp
SRCS += ./src/Rotary.cpp

OBJS = $(SRC:.cpp=.o)

INCDIR = -I./src

LIBDIR = 

LIBS = 

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LIBDIR) $(LIBS)

$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) $(INCDIR) -c $(SRCS)

all: clean $(OBJS) $(TARGET)

clean:
	-rm -f $(OBJS) $(TARGET) *.d