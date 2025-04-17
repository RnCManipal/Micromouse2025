#include "data_structures.h" // Include the header!
#include <Arduino.h>       // Include Arduino.h if you use Serial

// Queue Class Method Implementations
Queue::Queue() : first(0), last(0) {}

bool Queue::empty() {
    return first == last;
}

bool Queue::full() {
    return (last + 1) % SIZE == first;
}

void Queue::push(short int x, short int y) {
    if (full()) {
        return;
    }
    queue[last][0] = x;
    queue[last][1] = y;
    last = (last + 1) % SIZE;
}

 short int* Queue::pop() {
     if (empty()) {
         return nullptr;
     }
     short int* temp = queue[first];
     first = (first + 1) % SIZE;
     return temp;
 }

void Queue::display() {
    if (empty()) {
        return;
    }
    if (last > first) {
        for (int i = first; i < last; i++) {
            Serial.print(queue[i][0]);
            Serial.print(" ");
            Serial.println(queue[i][1]);
        }
    } else {
        int i = first;
        while (i < SIZE) {
            Serial.print(queue[i][0]);
            Serial.print(" ");
            Serial.println(queue[i][1]);
            i++;
        }
        i = 0;
        while (i < last) {
            Serial.print(queue[i][0]);
            Serial.print(" ");
            Serial.println(queue[i][1]);
            i++;
        }
    }
}

// Map Class Method Implementations
Map::Map() : size(0) {}

void Map::put(int key, int value) {
    if (size == MAX_SIZE) {
        Serial.println("Map is full!");
        return;
    }
    KeyValuePair* pair = new KeyValuePair;
    pair->key = key;
    pair->value = value;
    pairs[size] = pair;
    size++;
}

int Map::get(int key) const {
    for (int i = 0; i < size; i++) {
        if (pairs[i]->key == key) {
            return pairs[i]->value;
        }
    }
    Serial.println("Key not found in map!");
    return -1;
}

void Map::update(int key, int value) {
    for (int i = 0; i < size; i++) {
        if (pairs[i]->key == key) {
            pairs[i]->value = value;
            return;
        }
    }
    Serial.println("Key not found in map!");
}

Map::~Map() {
    for (int i = 0; i < size; i++) {
        delete pairs[i];
    }
}
