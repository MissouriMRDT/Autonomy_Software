\page CONTRIBUTING Contributing Guide

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
- Please provide as much context as possible and describe the *reproduction steps* that someone else can follow to recreate the issue on their own. This usually includes your code. For good bug reports you should isolate the problem and create a reduced test case.
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

## Styleguides
### Variable Names

Variable names should be in camel case with the lower case portion being the identifer from below. The name should give a good description of what the variable is for but should not excede 32-48 characters.


#### Member and Global Variables
- **Member Variables** should always start with `m_` and then use the following standards to set the rest of their name.
- **Global Variables** should always start with `g_` and then use the following standards to set the rest of their name.

Examples: `int g_nExampleGlobalInteger` or `int m_nExampleMemberInteger`

#### Signed Types
- Signed Integer > `n` > Example: `int nExampleSignedInteger`
- Signed Long    > `l` > Example: `long lExampleSignedLong`
- Signed Short   > `s` > Example: `short sExampleSignedShort`
- Signed Float   > `f` > Example: `float fExampleSignedFloat`
- Signed Double  > `d` > Example: `double dExampleSignedDouble`

#### Unsigned Types
- Unsigned Integer > `un` > Example: `unsigned int unExampleUnsignedInteger`
- Unsigned Long    > `ul` > Example: `unsigned long ulExampleUnsignedLong`
- Unsigned Short   > `us` > Example: `unsigned short usExampleUnsignedShort`
- Unsigned Float   > `uf` > Example: `unsigned float ufExampleUnsignedFloat`
- Unsigned Double  > `ud` > Example: `unsigned double udExampleUnsignedDouble`

#### Other Types
- Boolean   > `b`  > Example: `bool bExampleBoolean`
- Character > `c`  > Example: `char cExampleCharacter`
- Vector    > `v`  > Example: `vector<int> vExampleVector`
- Array     > `a`  > Example: `int aExampleArray[4]`
- Deque     > `dq` > Example: `deque dqExampleDeque`
- String    > `sz` > Example: `string szExampleString`
- Pointer   > `p`  > Example: `string* pExamplePointer`
- Enum      > `e`  > Example: `ExampleEnum eExampleUseOfEnum`
- Time      > `tm` > Example: `time_t tmExampleUseOfTime`
- Tuple     > `tp` > Example: `tuple tpExampleUseOfTuple`
- Thread    > `th` > Example: `jthread thExampleUseOfThread`
- Mutex     > `mu` > Example: `mutex muExampleUseOfMutex`
- Struct    > `st` > Example: `StructName stExampleUseOfStruct`

#### External Types
- OpenCV > `cv` > Example: `cv::Mat cvExampleMat`
- ZEDSDK > `sl` > Example: `sl::Mat slExampleZedMat`
- Boost:
    - State Chart > `sc`  > Example: `sc::simple_state<IdleState, StateMachine>`
- Quill  > `q` > Example: `quill::Config qConfig`

### Macro Names
Macros should be in all caps using underscores to seperate words. They should be detailed enough to not require an additional comment.

Example:`#define ARUCO_DEBUG_PRINTS false` or `#define AUTONOMY_MAJOR_VERSION 24`

### Function Names
Functions should use Pascal Case for names. They should use detailed names but also include a doxgen comment. Doxgen comments can be created by typing `/** <enter>`.

### Class Names
Class should use Pascal Case for names. They should use short names but also include a doxgen comment. Doxgen comments can be created by typing `/** <enter>`.

### Struct Names
Struct should use Pascal Case for names. They should use short names but also include a doxgen comment. Doxgen comments can be created by typing `/** <enter>`.

## Join The Team
Do you go to the Missouri University of Science and Technology? Are you interested in contributing and being a part of our team? If so go to [design.mst.edu](https://design.mst.edu) to learn how to sign up today!
