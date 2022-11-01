CC = g++

CFLAGS = -Wall

TARGET = soma_rpi

SRCS = ./main.cpp
SRCS += ./Clutch.cpp
SRCS += ./Rotary.cpp

OBJS = $(SRCS:%.cpp=%.o)

INCDIR = 

LIBDIR = 

LIBS = 

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LIBDIR) $(LIBS)

$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) $(INCDIR) -c $(SRCS)

all: clean $(OBJS) $(TARGET)

clean:
	-rm -f $(OBJS) $(TARGET) *.d