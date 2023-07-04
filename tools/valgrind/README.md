# Valgrind Leak Suppression and .supp Files

This directory contains suppression files for the Valgrind memory leak detection tool. These suppression files play a crucial role in ignoring leaks that are outside our control, improving the debugging process and focusing on our project-specific memory issues.

## Why .supp Files?

Valgrind is a powerful tool for detecting memory leaks and other memory-related issues in programs. However, when working with third-party libraries or frameworks like OpenCV, it is common to encounter leaks that are not directly under our control. These leaks might be harmless or have been addressed by the library maintainers in subsequent versions.

Suppressing these known leaks using `.supp` files allows us to filter out irrelevant memory leak reports during our project's execution with Valgrind. By excluding these leaks, we can concentrate on identifying and fixing memory issues within our own codebase more effectively.

## Valgrind Script

To run Valgrind with the `.supp` files, a script is provided within the Tools folder. This script executes the Valgrind tool, taking into account the suppression files to ignore specific leaks. The script ensures that only relevant memory leak reports related to our project are displayed, simplifying the debugging process.

Running the Valgrind script involves executing the following command in the terminal:

```
./run_valgrind.sh
```

NOTE: You may need to give the script executable permissions first. To do this run the command below:

```
sudo chmod +x ./run_valgrind.sh
```

## Future Proofing

The shell script located in this directory is written so that any `.supp` file is detected and applied to valgrind. To add more suppressions for other
libraries simply copy your `.supp` files to this folder. Some libraries will provide suppression files.

## Conclusion

The `.supp` files and the Valgrind script play an important role in our project's memory leak detection process. By using suppression files, we can exclude known leaks from external libraries, such as OpenCV, when running Valgrind. This allows us to focus on our project-specific memory issues, simplifying the debugging process and allowing us to more easily improve our code without bad external library code getting in the way.