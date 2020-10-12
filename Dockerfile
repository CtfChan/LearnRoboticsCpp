# syntax=docker/dockerfile:experimental

FROM conanio/gcc8 AS conan_deps

RUN mkdir -p /home/conan/LearnRoboticsCpp/build
COPY conanfile.txt /home/conan/LearnRoboticsCpp/conanfile.txt
COPY conan_profile /home/conan/LearnRoboticsCpp/conan_profile

RUN --mount=type=cache,target=~/.conan/data \
    cd /home/conan/LearnRoboticsCpp/build && conan install .. -pr=../conan_profile

FROM conan_deps AS repo_files

COPY CMakeLists.txt /home/conan/LearnRoboticsCpp/
COPY examples /home/conan/LearnRoboticsCpp/examples
COPY include /home/conan/LearnRoboticsCpp/include
COPY misc /home/conan/LearnRoboticsCpp/misc
COPY src /home/conan/LearnRoboticsCpp/src
COPY tests /home/conan/LearnRoboticsCpp/tests

FROM repo_files AS repo_build

RUN cd /home/conan/LearnRoboticsCpp/build && cmake .. && make -j 4

RUN cd /home/conan/LearnRoboticsCpp/build && ctest


VOLUME "/home/conan/LearnRoboticsCpp/"

WORKDIR "/home/conan/LearnRoboticsCpp/"
