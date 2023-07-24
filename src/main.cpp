/******************************************************************************
 * @brief Main program file. Sets up classes and runs main program functions.
 *
 * @file main.cpp
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "./AutonomyGlobals.h"

char* create_banner()
{
    const char* user = getenv("USER");
    size_t len       = 1 + 2 * 4 + strlen(user) + 1;
    char* b          = (char*) malloc(len);
    sprintf(b, "\t|** %s **|", user);
    return b;
}

void output_report(int nr)
{
    char* banner = create_banner();
    puts(banner);
    printf("Number: %d\n", nr);
    printf("\n");
}

/******************************************************************************
 * @brief Autonomy main function.
 *
 * @return int - Exit status number.
 *
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-20
 ******************************************************************************/
int main()
{
    // Print Software Header
    std::ifstream fHeaderText("../src/util/ASCII/v24.txt");
    std::string szHeaderText;
    if (fHeaderText)
    {
        std::ostringstream pHeaderText;
        pHeaderText << fHeaderText.rdbuf();
        szHeaderText = pHeaderText.str();
    }

    std::cout << szHeaderText << std::endl;
    std::cout << "Copyright \u00A9 2023 - Mars Rover Design Team\n" << std::endl;

    // Initialize Loggers
    InitializeLoggers();

    // TODO: Initialize Threads

    // TODO: Initialize RoveComm

    // Create Memory Leak
    for (int i = 1; i <= 3; i++)
        output_report(i);

    return 0;
}
