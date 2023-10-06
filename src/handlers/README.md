\dir src/handlers

# Handlers Directory

The **Handlers** directory is dedicated to storing C++ files that handle the high-level management of other, globally shared objects. #include these handlers
to access things like cameras, tag detection, object detection, and drivers.

## Directory Structure

The **Handlers** directory stores files representing the main classes of different modules, each of which will handle one or many objects created from classes within this codebase. For example, the TagDetectionHandler may store multiple objects of the ArUcoDetector and TensorflowDetector classes. Each
of these classes achieve the same thing, but are initialized with different parameters (like different images to run detection on). The handler exists to
provide a way for other class to access the results of the tag detections for the different object (each which runs detection on a different image). 

## General Usage

Here's a general guideline for organizing files within the **Handlers** directory:

- Create a separate file for each handler that stores or contains instances of globally accessible classes that relate to each other.
- Use descriptive names for the files that reflect the purpose or functionality of each class.
- Ensure that the files are properly documented with comments explaining the class's role, its interaction with other modules, and any relevant details.
- Implement the necessary thread management mechanisms within each class so that other classes/threads can access pointers to these objects.
