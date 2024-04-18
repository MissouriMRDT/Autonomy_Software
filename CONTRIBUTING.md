\page md_CONTRIBUTING Contributing Guide

# Contributing to Autonomy Software

## Contributing

### Reporting Bugs

#### Before Submitting a Bug Report

A good bug report shouldn't leave others needing to chase you down for more information. Therefore, we ask you to investigate carefully, collect information and describe the issue in detail in your report. Please complete the following steps in advance to help us fix any potential bug as fast as possible.

- Make sure that you are using the latest version.
- Determine if your bug is really a bug and not an error on your side e.g. using incompatible environment components/versions (Make sure that you have read the [documentation](https://missourimrdt.github.io/Autonomy_Software/)).
- To see if other users have experienced (and potentially already solved) the same issue you are having, check if there is not already a bug report existing for your bug or error in the [bug tracker](https://github.com/MissouriMRDT/Autonomy_Software/issues).
- Collect information about the bug:
  - Backtrace
  - OS, Platform and Version
  - Can you reliably reproduce the issue? And can you also reproduce it with older versions?

#### How Do I Submit a Good Bug Report?

We use GitHub issues to track bugs and errors. If you run into an issue with the project:

- Open an [Issue](https://github.com/MissouriMRDT/Autonomy_Software/issues/new).
- Explain the behavior you would expect and the actual behavior.
- Please provide as much context as possible and describe the _reproduction steps_ that someone else can follow to recreate the issue on their own. This usually includes your code. For good bug reports you should isolate the problem and create a reduced test case.
- Provide the information you collected in the previous section.

### Suggesting Enhancements

This section guides you through submitting an enhancement suggestion for Autonomy Software, **including completely new features and minor improvements to existing functionality**. Following these guidelines will help maintainers and the community to understand your suggestion and find related suggestions.

#### Before Submitting an Enhancement

- Make sure that you are using the latest version.
- Read the [documentation](https://missourimrdt.github.io/Autonomy_Software/) carefully and find out if the functionality is already covered.
- Perform a [search](https://github.com/MissouriMRDT/Autonomy_Software/issues) to see if the enhancement has already been suggested. If it has, add a comment to the existing issue instead of opening a new one.

#### How Do I Submit a Good Enhancement Suggestion?

Enhancement suggestions are tracked as [GitHub issues](https://github.com/MissouriMRDT/Autonomy_Software/issues).

- Use a **clear and descriptive title** for the issue to identify the suggestion.
- Provide a **step-by-step description of the suggested enhancement** in as many details as possible.
- **Describe the current behavior** and **explain which behavior you expected to see instead** and why.

### Commit Messages

Commit names should be no more than 50 characters. They should have `#<issue num>` to link commits to the issue.

### Pull Requests

Pull Request names should be no more than 50 characters. The description should be quite detailed in everything that was added and should have `closed #<issue num>` to mark that the pull request will fix an issue.

## Marking Issues in Code

### Utilizing TODO-Trees

A list of action items can be built in the TODO extension tab using these keywords to highlight and mark sections of code for later review:
- // * @todo Implement the behavior specific to the Approaching Object state
- // @todo Implement the behavior specific to the Approaching Object state
- // TODO: Implement the behavior specific to the Approaching Object state
- // BUG: This breaks occasionally.
- // HACK: This is bad, never do this.
- // FIXME: Broken
- // LEAD: Check this out...
- // ISSUE NOTE: This is messed up.
- // TEST: Make sure this works, write unit tests.
- // [ ] This is a list item.
- // [x] This is a checked list item.

## Styleguides

### Variable Names

Variable names should be in camel case with the lower case portion being the identifier from below. The name should give a good description of what the variable is for but should not exceed 32-48 characters.

#### Member and Global Variables

- **Member Variables** should always start with `m_` and then use the following standards to set the rest of their name.
- **Global Variables** should always start with `g_` and then use the following standards to set the rest of their name.

Examples: `int g_nExampleGlobalInteger` or `int m_nExampleMemberInteger`

#### Signed Types

- Signed Integer > `n` > Example: `int nExampleSignedInteger`
- Signed Long > `l` > Example: `long lExampleSignedLong`
- Signed Short > `s` > Example: `short sExampleSignedShort`
- Signed Float > `f` > Example: `float fExampleSignedFloat`
- Signed Double > `d` > Example: `double dExampleSignedDouble`

#### Unsigned Types

- Unsigned Char > `uc` > Example: `unsigned char ucExampleUnsignedChar`
- Unsigned Integer > `un` > Example: `unsigned int unExampleUnsignedInteger`
- Unsigned Long > `ul` > Example: `unsigned long ulExampleUnsignedLong`
- Unsigned Short > `us` > Example: `unsigned short usExampleUnsignedShort`
- Unsigned Float > `uf` > Example: `unsigned float ufExampleUnsignedFloat`
- Unsigned Double > `ud` > Example: `unsigned double udExampleUnsignedDouble`

#### Other Types

- Boolean > `b` > Example: `bool bExampleBoolean`
- Character > `c` > Example: `char cExampleCharacter`
- Vector > `v` > Example: `vector<int> vExampleVector`
- Array > `a` > Example: `int aExampleArray[4]`
- Deque > `dq` > Example: `deque dqExampleDeque`
- Queue > `q` > Example: `queue<int> qExampleQueue`
- Iterator > `it` > Example: `iterator itExampleIterator`
- String > `sz` > Example: `string szExampleString`
- Pointer > `p` > Example: `string* pExamplePointer`
- Enum > `e` > Example: `ExampleEnum::eExampleUseOfEnum`
- Time > `tm` > Example: `time_t tmExampleUseOfTime`
- Template > `t` > Example: `T tExampleUseOfTemplateType`
- Size > `si` > Example: `size_t siExampleUseOfTime`
- Tuple > `tp` > Example: `tuple tpExampleUseOfTuple`
- STD Type > `std` > Example: `memory_order stdExampleUseOfMemoryOrder`
- Thread > `th` > Example: `jthread thExampleUseOfThread`
- Mutex > `mu` > Example: `mutex muExampleUseOfMutex`
- Lock > `lk` > Example: `lock lkExampleUseOfLock`
- Condition Variable > `cd` > Example: `condition_variable cdExampleUseOfConditionVariable`
- Struct > `st` > Example: `StructName stExampleUseOfStruct`
- Future > `fu` > Example: `future<void> fuExampleUseOfFuture`
- Promise > `pm` > Example: `promise<void> pmExampleUseIfPromise`
- Unordered Map > `um` > Example: `unordered_map<int, int> umExampleUseOfUnorderedMap`

#### External Types

- OpenCV > `cv` > Example: `cv::Mat cvExampleMat`
- ZEDSDK > `sl` > Example: `sl::Mat slExampleZedMat`
- Quill > `q` > Example: `quill::Config qConfig`
- GeographicLib > `ge` > Example: `GeographicLib::Geodesic geExampleGeographicType`
- Tensorflow > `tf` > Example: `tflite::Interpreter tfExampleTensorflowType`
- Libedgetpu > `tpu` > Example: `edgetpu::EdgeTpuManager tpuExampleEdgeTPUType`

### Macro Names

Macros should be in all caps using underscores to separate words. They should be detailed enough to not require an additional comment.

Example:`#define ARUCO_DEBUG_PRINTS false` or `#define AUTONOMY_MAJOR_VERSION 24`

### Function Names

Functions should use Pascal Case for names. They should use detailed names but also include a doxygen comment. Doxygen comments can be created by typing `/** <enter>`.

### Class Names

Class should use Pascal Case for names. They should use short names but also include a Doxygen comment. Doxygen comments can be created by typing `/** <enter>`.

### Struct Names

Struct should use Pascal Case for names. They should use short names but also include a Doxygen comment. Doxygen comments can be created by typing `/** <enter>`.

## Join The Team

Do you go to the Missouri University of Science and Technology? Are you interested in contributing and being a part of our team? If so go to [design.mst.edu](https://design.mst.edu) to learn how to sign up today!
