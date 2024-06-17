#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <cstdlib>

int main() {
    pid_t pid1, pid2;

    // Create the first child process
    pid1 = fork();

    if (pid1 < 0) {
        std::cerr << "Fork failed for first child!" << std::endl;
        exit(1);
    } else if (pid1 == 0) {
        // First child process
        std::cout << "First child process with PID " << getpid() << std::endl;
        while (true) {
            // Simulating some work
            std::cout << "First child working..." << std::endl;
            sleep(2);
        }
    } else {
        // Create the second child process
        pid2 = fork();

        if (pid2 < 0) {
            std::cerr << "Fork failed for second child!" << std::endl;
            exit(1);
        } else if (pid2 == 0) {
            // Second child process
            std::cout << "Second child process with PID " << getpid() << std::endl;
            // Increase the priority of this child process
            if (setpriority(PRIO_PROCESS, 0, -10) < 0) {
                std::cerr << "Failed to set priority for second child!" << std::endl;
            } else {
                std::cout << "Priority of second child increased." << std::endl;
            }
            while (true) {
                // Simulating some work
                std::cout << "Second child working..." << std::endl;
                sleep(2);
            }
        } else {
            // Father process
            std::cout << "Father process with PID " << getpid() << std::endl;
            // Wait for both child processes to finish
            wait(NULL);
            wait(NULL);
        }
    }

    return 0;
}