FROM conanio/gcc8 AS conan_deps

RUN mkdir -p /home/conan/conan_gtest/build
COPY conanfile.txt /home/conan/conan_gtest/conanfile.txt
COPY conan_profile /home/conan/conan_gtest/conan_profile

RUN cd /home/conan/conan_gtest/build && conan install .. -pr=../conan_profile

FROM conan_deps AS repo_files

COPY CMakeLists.txt /home/conan/conan_gtest/
COPY tests /home/conan/conan_gtest/tests
COPY libbar /home/conan/conan_gtest/libbar

FROM repo_files AS repo_build

RUN cd /home/conan/conan_gtest/build && cmake .. && make -j 4

RUN cd /home/conan/conan_gtest/build && ctest


VOLUME "/home/conan/conan_gtest/"

WORKDIR "/home/conan/conan_gtest/"
