#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <cstring>
#include <cstdlib>

void childProcess(int pipeFd[2], const std::string &message) {
    close(pipeFd[0]); // Close the read end of the pipe
    while (true) {
        write(pipeFd[1], message.c_str(), message.size() + 1); // Send message to the father process
        sleep(2); // Simulate some work
    }
}

void fatherProcess(int pipeFd1[2], int pipeFd2[2]) {
    close(pipeFd1[1]); // Close the write end of the first pipe
    close(pipeFd2[1]); // Close the write end of the second pipe

    char buffer1[256];
    char buffer2[256];

    while (true) {
        // Read messages from both child processes
        read(pipeFd1[0], buffer1, sizeof(buffer1));
        read(pipeFd2[0], buffer2, sizeof(buffer2));

        std::cout << "Father received from first child: " << buffer1 << std::endl;
        std::cout << "Father received from second child: " << buffer2 << std::endl;

        sleep(2); // Simulate some work
    }
}

int main() {
    int pipeFd1[2], pipeFd2[2];

    if (pipe(pipeFd1) < 0 || pipe(pipeFd2) < 0) {
        std::cerr << "Pipe creation failed!" << std::endl;
        exit(1);
    }
    
    pid_t pid1 = fork();
    std::cout<<pid1<<std::endl;
    if (pid1 < 0) {
        std::cerr << "Fork failed for the first child!" << std::endl;
        exit(1);
    } else if (pid1 == 0) {
        // First child process
        std::cout << "First child process with PID " << getpid() << std::endl;
        childProcess(pipeFd1, "Message from first child");
    } else {
        pid_t pid2 = fork();

        if (pid2 < 0) {
            std::cerr << "Fork failed for the second child!" << std::endl;
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
            childProcess(pipeFd2, "Message from second child");
        } else {
            // Father process
            std::cout << "Father process with PID " << getpid() << std::endl;
            fatherProcess(pipeFd1, pipeFd2);

            // Wait for both child processes to finish
            wait(NULL);
            wait(NULL);
        }
    }

    return 0;
}


















// compile single file with g++ example_gpt.cpp -o example_gpt

#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstring>

int main() {
    int pipe1[2];
    int pipe2[2];
    char message1[] = "Parent says hi!";
    char message2[100] = "Child 2 says hi!";

    if (pipe(pipe1) == -1 || pipe(pipe2) == -1) {
        std::cerr << "Failed to create pipe." << std::endl;
        return 1;
    }

    pid_t child1Pid, child2Pid;

    child1Pid = fork();

    if (child1Pid == 0) {
        // Child 1 process code
        std::cout << "Hello from the child process 1!\n";

        close(pipe1[1]);  // Close the write end of pipe1
        // close(pipe2[0]);  // Close the read end of pipe2

        char buffer[100];
        read(pipe1[0], buffer, sizeof(buffer));

        std::cout << "Child 1 received message: " << buffer << std::endl;

        close(pipe1[0]);  // Close the read end of pipe1
        // close(pipe2[1]);  // Close the write end of pipe2

        return 0;
    } else if (child1Pid > 0) {
        child2Pid = fork();

        if (child2Pid == 0) {
            // Child 2 process code
            std::cout << "Hello from the child process 2!\n";

            // close(pipe1[0]);  // Close the read end of pipe1
            close(pipe2[0]);  // Close the read end of pipe2

            write(pipe2[1], message2, strlen(message2) + 1);

            // close(pipe1[1]);  // Close the write end of pipe1
            close(pipe2[1]);  // Close the write end of pipe2

            return 0;
        } else if (child2Pid > 0) {
            // Parent process code
            close(pipe1[0]);  // Close the read end of pipe1
            close(pipe2[1]);  // Close the write end of pipe2

            write(pipe1[1], message1, strlen(message1) + 1);

            char buffer[100];
            read(pipe2[0], buffer, sizeof(buffer));

            std::cout << "Parent received message: " << buffer << std::endl;

            // close(pipe1[1]);  // Close the write end of pipe1
            // close(pipe2[1]);  // Close the write end of pipe2

            wait(NULL);  // Wait for child 1 to finish
            wait(NULL);  // Wait for child 2 to finish

            return 0;
        } else {
            std::cerr << "Failed to create child process 2." << std::endl;
            return 1;
        }
    } else {
        std::cerr << "Failed to create child process 1." << std::endl;
        return 1;
    }
}
