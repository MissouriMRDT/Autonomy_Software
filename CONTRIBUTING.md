\page md_CONTRIBUTING Contributing Guide

# Contributing to Autonomy Software

## 📋 Overview

Thank you for considering contributing to Autonomy Software! This guide will help you report bugs, suggest enhancements, and follow our code style and conventions.

---

## 🐛 Reporting Bugs

### 🔍 Before Submitting a Bug Report

To help us fix potential bugs quickly, please ensure the following:

- **Use the latest version** of the software.
- **Verify** that the issue is a bug and not an error on your side (e.g., incompatible environment components/versions). Check the [documentation](https://missourimrdt.github.io/Autonomy_Software/) for guidance.
- **Search** the [bug tracker](https://github.com/MissouriMRDT/Autonomy_Software/issues) to see if the issue has already been reported or solved.
- **Collect relevant information** such as:
  - Backtrace (if applicable)
  - OS, Platform, and Version
  - Steps to reliably reproduce the issue

### ✍️ Submitting a Good Bug Report

To submit a bug report:

1. **Open a new [Issue](https://github.com/MissouriMRDT/Autonomy_Software/issues/new)** on GitHub.
2. **Describe the expected behavior** and **the actual behavior**.
3. **Provide detailed reproduction steps**, including code snippets where possible.
4. **Include any gathered information** (backtrace, OS, platform, version).

Good bug reports help us isolate the problem faster and improve the software more efficiently.

---

## 💡 Suggesting Enhancements

### 🔍 Before Submitting an Enhancement Suggestion

- **Ensure you are using the latest version** of the software.
- **Check the [documentation](https://missourimrdt.github.io/Autonomy_Software/)** to see if the feature already exists.
- **Search the [issue tracker](https://github.com/MissouriMRDT/Autonomy_Software/issues)** to see if someone has already suggested a similar enhancement.

### ✍️ Submitting a Good Enhancement Suggestion

To submit an enhancement suggestion:

1. **Open a new [Issue](https://github.com/MissouriMRDT/Autonomy_Software/issues)** on GitHub.
2. **Use a clear, descriptive title**.
3. **Provide a step-by-step description** of your suggested enhancement.
4. **Explain the current behavior** and **describe the expected behavior**, including why the change is beneficial.

---

## 💾 Commit Messages

### Guidelines

- Keep commit messages under **50 characters**.
- Always include a reference to the relevant issue number in the format: `#<issue num>`.

Example:
```
fix: Resolve crash in navigation state machine #42
```

---

## 🔄 Pull Requests

### Guidelines

- Keep pull request titles under **50 characters**.
- Include a detailed description of the changes made.
- Use `closed #<issue num>` in the description to link the pull request to a specific issue.

Example:
```
Closed #42: Fixed crash in the navigation state machine by adjusting state transitions.
```

---

## 🛑 Marking Issues in Code

To mark sections of code for later review, we use keywords that can be compiled into a to-do list in the TODO extension tab. Here's a list of commonly used keywords:

- `// * @todo` or `// @todo` – Marks sections of code for future implementation.
- `// TODO:` – Generic to-do items.
- `// BUG:` – Known issues or bugs.
- `// HACK:` – Temporary workarounds that need improvement.
- `// FIXME:` – Sections of code that are broken and need fixing.
- `// LEAD:` – Important notes for review by team leads.
- `// ISSUE NOTE:` – Problematic code that needs attention.
- `// TEST:` – Indicates areas that require testing.

You can also use checklist-style comments:
- `// [ ]` – Unchecked to-do item.
- `// [x]` – Checked to-do item.

---

---

## 🎨 Style Guide

### 🔢 Signed and Unsigned Types

The following tables outline the naming conventions for signed and unsigned types:

#### Signed Types

| Type              | Prefix | Example                          |
|-------------------|--------|----------------------------------|
| Signed Integer    | `n`    | `int nExampleSignedInteger`      |
| Signed Long       | `l`    | `long lExampleSignedLong`        |
| Signed Short      | `s`    | `short sExampleSignedShort`      |
| Signed Float      | `f`    | `float fExampleSignedFloat`      |
| Signed Double     | `d`    | `double dExampleSignedDouble`    |

#### Unsigned Types

| Type               | Prefix | Example                           |
|--------------------|--------|-----------------------------------|
| Unsigned Char      | `uc`   | `unsigned char ucExampleUnsignedChar` |
| Unsigned Integer   | `un`   | `unsigned int unExampleUnsignedInteger` |
| Unsigned Long      | `ul`   | `unsigned long ulExampleUnsignedLong` |
| Unsigned Short     | `us`   | `unsigned short usExampleUnsignedShort` |
| Unsigned Float     | `uf`   | `unsigned float ufExampleUnsignedFloat` |
| Unsigned Double    | `ud`   | `unsigned double udExampleUnsignedDouble` |

---

### 📚 Other Types

The table below lists various other common types and their corresponding prefixes:

| Type                | Prefix  | Example                                |
|---------------------|---------|----------------------------------------|
| Boolean             | `b`     | `bool bExampleBoolean`                 |
| Character           | `c`     | `char cExampleCharacter`               |
| Vector              | `v`     | `vector<int> vExampleVector`           |
| Array               | `a`     | `int aExampleArray[4]`                 |
| Deque               | `dq`    | `deque dqExampleDeque`                 |
| Queue               | `q`     | `queue<int> qExampleQueue`             |
| Iterator            | `it`    | `iterator itExampleIterator`           |
| String              | `sz`    | `string szExampleString`               |
| Pointer             | `p`     | `string* pExamplePointer`              |
| Enum                | `e`     | `ExampleEnum::eExampleUseOfEnum`       |
| Time                | `tm`    | `time_t tmExampleUseOfTime`            |
| Template            | `t`     | `T tExampleUseOfTemplateType`          |
| Size                | `si`    | `size_t siExampleUseOfSize`            |
| Tuple               | `tp`    | `tuple tpExampleUseOfTuple`            |
| STD Type            | `std`   | `memory_order stdExampleUseOfMemoryOrder` |
| Thread              | `th`    | `jthread thExampleUseOfThread`         |
| Mutex               | `mu`    | `mutex muExampleUseOfMutex`            |
| Lock                | `lk`    | `lock lkExampleUseOfLock`              |
| Condition Variable  | `cd`    | `condition_variable cdExampleUseOfConditionVariable` |
| Struct              | `st`    | `StructName stExampleUseOfStruct`      |
| Future              | `fu`    | `future<void> fuExampleUseOfFuture`    |
| Promise             | `pm`    | `promise<void> pmExampleUseOfPromise`  |
| Unordered Map       | `um`    | `unordered_map<int, int> umExampleUseOfUnorderedMap` |

---

### 🧩 External Types

External libraries and their corresponding type prefixes are listed below:

| Library        | Prefix | Example                                      |
|----------------|--------|----------------------------------------------|
| OpenCV         | `cv`   | `cv::Mat cvExampleMat`                       |
| ZED SDK        | `sl`   | `sl::Mat slExampleZedMat`                    |
| Quill          | `q`    | `quill::Config qConfig`                      |
| GeographicLib  | `ge`   | `GeographicLib::Geodesic geExampleGeographicType` |
| Tensorflow     | `tf`   | `tflite::Interpreter tfExampleTensorflowType` |
| Libedgetpu     | `tpu`  | `edgetpu::EdgeTpuManager tpuExampleEdgeTPUType` |

---

### 🔠 Macro Names

Macros should be in all caps using underscores to separate words. They should be detailed enough to not require an additional comment.

Example:
```c
#define ARUCO_DEBUG_PRINTS false
#define AUTONOMY_MAJOR_VERSION 24
```

---

### 📐 Function Names

Functions should use PascalCase for names. Use detailed names and include a Doxygen comment.

Example:
```cpp
/* Type: `/**` and then <enter> to start doxygen comment block */
void InitializeSensor();
```

---

### 🏷️ Class and Struct Names

Classes and structs should use PascalCase for names. Use short, descriptive names, and include a Doxygen comment.

Example for a class:

```cpp
/* Type: `/**` and then <enter> to start doxygen comment block */
class SensorManager {
  // class implementation
};

```

---

## 👥 Join the Team

Do you attend Missouri University of Science and Technology? Are you interested in contributing to the Mars Rover Design Team and working on autonomy software? Head to [https://design.mst.edu](https://design.mst.edu) to learn how to join us today!
