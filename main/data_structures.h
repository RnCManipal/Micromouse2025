#ifndef DATA_STRUCTURES_H_
#define DATA_STRUCTURES_H_

#include <Arduino.h>

#define MAX_SIZE 4
#define SIZE 260

// Queue Class
class Queue {
private:
    short int queue[SIZE][2];
    int first, last;

public:
    Queue();
    bool empty();
    bool full();
    void push(short int x, short int y);
    short int* pop();
    void display();
};

// Map Class
class Map {
private:
    struct KeyValuePair {
        int key;
        int value;
    };

    KeyValuePair* pairs[MAX_SIZE];
    int size;

public:
    Map();
    void put(int key, int value);
    int get(int key) const;
    void update(int key, int value);
    ~Map();
};

#endif