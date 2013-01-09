TARGET=main

all: $(TARGET)

objects: *.cpp *.h
		g++ -c *.cpp

$(TARGET): objects
		g++ *.o -o $(TARGET) -pthread -lm

clean:
	rm *.o $(TARGET)

.PHONY: all clean
