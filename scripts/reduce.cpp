#include <stdio.h>
#include <string.h>

#define MAX 1000

// Function to check if two directions are opposite
int isOpposite(char a, char b) {
    return (a == 'N' && b == 'S') ||
           (a == 'S' && b == 'N') ||
           (a == 'E' && b == 'W') ||
           (a == 'W' && b == 'E');
}

// Function to reduce directions
void reduceDirections(const char* input) {
    char tempInput[MAX];
    strncpy(tempInput, input, MAX);
    
    char directions[MAX][2];  // Array of single-char strings
    int count = 0;

    // Split the input string into individual directions
    char *token = strtok(tempInput, " ");
    while (token != NULL) {
        strcpy(directions[count++], token);
        token = strtok(NULL, " ");
    }

    // Use stack logic to reduce directions
    char stack[MAX][2];
    int top = -1;

    for (int i = 0; i < count; i++) {
        if (top >= 0 && isOpposite(stack[top][0], directions[i][0])) {
            top--;  // Cancel out opposite direction
        } else {
            strcpy(stack[++top], directions[i]);
        }
    }

    // Print the reduced result
    printf("Reduced directions:\n");
    for (int i = 0; i <= top; i++) {
        printf("%s ", stack[i]);
    }
    printf("\n");
}

int main() {
    const char *input = "N N E N N N E E S W E E S W ";
    reduceDirections(input);
    return 0;
}