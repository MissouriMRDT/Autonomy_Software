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

## HOW TO INTERPRET VALGRIND OUTPUT

The output of valgrind will likely never show `All heap blocks were freed -- no leaks are possible` unless all of our external libraries fix their code. So valgrinds output with suppressions might look a little different then you're used to. Don't worry though, as long as all of the leaked memory is within the suppressed category, we're fine. The example below shows a compile of the Autonomy_Software code that doesn't leak any additional memory.

```
==23697== HEAP SUMMARY:
==23697==     in use at exit: 159,937 bytes in 239 blocks
==23697==   total heap usage: 25,723 allocs, 25,484 frees, 2,318,129 bytes allocated
==23697== 
==23697== LEAK SUMMARY:
==23697==    definitely lost: 0 bytes in 0 blocks
==23697==    indirectly lost: 0 bytes in 0 blocks
==23697==      possibly lost: 0 bytes in 0 blocks
==23697==    still reachable: 0 bytes in 0 blocks
==23697==         suppressed: 159,937 bytes in 239 blocks
==23697== 
==23697== ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 4 from 4)
--23697-- 
--23697-- used_suppression:     34 dl_init /workspaces/Autonomy_Software/tools/valgrind/opencv_3rdparty.supp:103 suppressed: 158,413 bytes in 223 blocks
--23697-- used_suppression:     15 OpenCV-SingletonLogger /workspaces/Autonomy_Software/tools/valgrind/opencv.supp:222 suppressed: 1,508 bytes in 15 blocks
--23697-- used_suppression:      1 dl_open /workspaces/Autonomy_Software/tools/valgrind/opencv_3rdparty.supp:110 suppressed: 16 bytes in 1 blocks
--23697-- used_suppression:      4 dl-hack4-64bit-addr-1 /usr/lib/x86_64-linux-gnu/valgrind/default.supp:1277
```

## Future Proofing

The shell script located in this directory is written so that any `.supp` file is detected and applied to valgrind. To add more suppressions for other
libraries simply copy your `.supp` files to this folder. Some libraries will provide suppression files.

## Conclusion

The `.supp` files and the Valgrind script play an important role in our project's memory leak detection process. By using suppression files, we can exclude known leaks from external libraries, such as OpenCV, when running Valgrind. This allows us to focus on our project-specific memory issues, simplifying the debugging process and allowing us to more easily improve our code without bad external library code getting in the way.