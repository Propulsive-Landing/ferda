# Code in this folder was generated using Simulink shared library generation with Embeded coder

[Reference 1](https://www.mathworks.com/help/rtw/ug/relocate-code-to-another-development-environment.html)
[Reference 2](https://www.mathworks.com/help/ecoder/ug/deploy-component-algorithm-as-component-model-library-by-using-cmake.html)
[Reference 3](https://www.mathworks.com/help/ecoder/ug/configure-cmake-build-process.html)

To build from artifacts, two folders are generated. One is the source which must be specified in "pathToSource" while the other are the matlab dependencies. The matlab dependencies folder is generally larger, and has a lot of sub-directories. 

cmake -S pathToSource -B pathToProposedLocationOfDerivedFiles 
-DMATLAB_ROOT=pathToUnzippedmlrFiles