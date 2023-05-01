# 14744-project
This program is written by Yifan Lan (Andrew ID: yifanlan) for the final project of the course 14744. More information can be seen from the project journal: https://docs.google.com/document/d/1ZLyOWhyvINdyuoEdPHG9UoeRC2rAOJoE0WPCF0KSXds/edit?usp=sharing
```
docker pull movesense/sensor-build-env:latest
cd movesense-device-lib
docker run -it --rm -v C:\Users\lanyifan\Documents\GitHub\14744-project\movesense-device-lib:/movesense:delegated movesense/sensor-build-env:latest
cd /movesense
mkdir myBuild
cd myBuild
cmake -G Ninja -DMOVESENSE_CORE_LIBRARY=../MovesenseCoreLib/ -DCMAKE_TOOLCHAIN_FILE=../MovesenseCoreLib/toolchain/gcc-nrf52.cmake ../myApp
ninja pkgs
```

