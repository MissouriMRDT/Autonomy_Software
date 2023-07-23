<details><summary></summary></details>
<p>
~Doxygen flag/marks~

\dir tools/valgrind
</p>

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

The output of valgrind will likely never show `All heap blocks were freed -- no leaks are possible` unless all of our external libraries fix their code. So valgrinds output with suppressions might look a little different then you're used to. Don't worry though, as long as all of the leaked memory is within the suppressed or still reachable category, we're fine. The example below shows a compile of the Autonomy_Software code that doesn't leak any additional memory.

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

## But, Why is This Okay?

There are multiple interpretations of the term "memory leak" that programmers commonly use. Primarily, two distinct definitions are prevalent in the programming community.

The first widely accepted definition of a "memory leak" states that it occurs when memory is allocated but not subsequently freed before the program terminates. However, some programmers argue, and rightfully so, that certain types of memory leaks falling under this definition do not pose any significant issues. Hence, they should not be considered true "memory leaks."

A more stringent and practical definition of a "memory leak" is as follows: memory is allocated, but it cannot be freed afterwards because the program no longer possesses any pointers to the allocated memory block. In essence, if there are no remaining pointers, the memory cannot be released, leading to a "memory leak." Valgrind adopts this stricter definition when referring to a "memory leak." Such leaks can potentially result in substantial depletion of the heap, particularly in long-lived processes.

Within Valgrind's leak report, the "still reachable" category encompasses allocations that align with only the first definition of a "memory leak." Although these blocks were not freed, they could have been freed if the programmer had desired, as the program was still maintaining pointers to those memory blocks.

In general, there is no need to be concerned about "still reachable" blocks, as they do not pose the same problems that true memory leaks can cause. For example, "still reachable" blocks typically do not lead to heap exhaustion. They are typically one-time allocations, and references to them are retained throughout the process's lifetime. All object pointers within Autonomy_Software should still be cleaned-up as good practice. The only memory that is "still reachable" after our program finished should be that of the 3rd party libraries we use.

## Future Proofing

The shell script located in this directory is written so that any `.supp` file is detected and applied to valgrind. To add more suppressions for other
libraries simply copy your `.supp` files to this folder. Some libraries will provide suppression files.

## Conclusion

The `.supp` files and the Valgrind script play an important role in our project's memory leak detection process. By using suppression files, we can exclude known leaks from external libraries, such as OpenCV, when running Valgrind. This allows us to focus on our project-specific memory issues, simplifying the debugging process and allowing us to more easily improve our code without bad external library code getting in the way.