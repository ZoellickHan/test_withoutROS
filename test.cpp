#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Define a struct with some fields
struct MyStruct {
    int a;
    float b;
    char c;
};

// Function to print the uint8_t array for verification
void print_uint8_array(uint8_t *array, size_t size) {
    for (size_t i = 0; i < size; i++) {
        printf("%02x ", array[i]);
    }
    printf("\n");
}

int main() {
    // Initialize the struct
    struct MyStruct myStruct = {42, 3.14f, 'x'};

    // Calculate the size of the struct
    size_t structSize = sizeof(myStruct);

    // Allocate a uint8_t array to hold the struct data
    uint8_t byteArray[structSize];

    // Copy the struct data to the uint8_t array
    memcpy(byteArray, &myStruct, structSize);

    // Print the uint8 array for verification
    print_uint8_array(byteArray, structSize);

    return 0;
}