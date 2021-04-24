

#ifndef WRITETOBINARY_T
#define WRITETOBINARY_T

#include <fstream>

MTS_NAMESPACE_BEGIN

template<typename T>
void writeToBinaryFile(const T* const data, size_t size, std::string path) {
    std::ofstream file(path, std::ios::binary);
    if (file.is_open()) {
  
        file.write(reinterpret_cast<const char* const>(data), size);
        file.close();
    } else {
        std::cout << "FAILED TO OPEN FILE" << std::endl;
    }
}

template<typename T>
size_t readBinaryFile(std::string path, T* data, size_t size) {
    std::ifstream file(path, std::ios::binary);
    if (file.is_open()) {
        file.seekg (0, std::ios::beg);
        file.read(reinterpret_cast<char*>(data), size);
        file.close();
        return size;
    } else {
        std::cout << "FAILED TO OPEN FILE" << std::endl;
        return -1;
    }
}

MTS_NAMESPACE_END

#endif