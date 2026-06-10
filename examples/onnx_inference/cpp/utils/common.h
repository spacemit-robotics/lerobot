/**
 * @file common.h
 * @brief Shared timing and NumPy .npy loading helpers for ACT ONNX C++ tools.
 */

#ifndef ONNX_INFERENCE_ACT_CPP_UTILS_COMMON_H_
#define ONNX_INFERENCE_ACT_CPP_UTILS_COMMON_H_

#include <chrono>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Timer {
    std::chrono::high_resolution_clock::time_point t0;

    void Start() { t0 = std::chrono::high_resolution_clock::now(); }

    double ElapsedMs() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now() - t0)
                   .count() /
               1000.0;
    }
};

/**
 * @brief Load a little-endian float32 NumPy .npy tensor.
 *
 * @param path Path to the .npy file.
 * @param shape Output tensor shape parsed from the .npy header.
 * @return Tensor values as a contiguous float vector.
 * @throws std::runtime_error if the file is missing or not float32 .npy.
 */
inline std::vector<float> LoadNpyF32(const std::string& path, std::vector<int64_t>& shape) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open npy: " + path);
    char magic[6];
    f.read(magic, 6);
    if (std::string(magic, 6) != std::string("\x93NUMPY", 6))
        throw std::runtime_error("not a npy file: " + path);
    uint8_t major = 0, minor = 0;
    f.read(reinterpret_cast<char*>(&major), 1);
    f.read(reinterpret_cast<char*>(&minor), 1);
    (void)minor;

    uint32_t hlen = 0;
    if (major == 1) {
        uint16_t hlen16 = 0;
        f.read(reinterpret_cast<char*>(&hlen16), sizeof(hlen16));
        hlen = hlen16;
    } else if (major == 2) {
        f.read(reinterpret_cast<char*>(&hlen), sizeof(hlen));
    } else {
        throw std::runtime_error("unsupported npy version: " + path);
    }
    std::string header(hlen, '\0');
    f.read(&header[0], hlen);
    if (header.find("'<f4'") == std::string::npos && header.find("\"<f4\"") == std::string::npos)
        throw std::runtime_error("npy dtype must be <f4 (float32): " + path);

    shape.clear();
    size_t sp = header.find("(", header.find("shape"));
    size_t ep = header.find(")", sp);
    std::string dims = header.substr(sp + 1, ep - sp - 1);
    size_t total = 1;
    for (size_t i = 0; i < dims.size();) {
        while (i < dims.size() && !std::isdigit(static_cast<unsigned char>(dims[i]))) i++;
        if (i >= dims.size()) break;
        int64_t v = 0;
        while (i < dims.size() && std::isdigit(static_cast<unsigned char>(dims[i]))) {
            v = v * 10 + (dims[i++] - '0');
        }
        shape.push_back(v);
        total *= v;
    }
    std::vector<float> data(total);
    f.read(reinterpret_cast<char*>(data.data()), total * sizeof(float));
    return data;
}

#endif  // ONNX_INFERENCE_ACT_CPP_UTILS_COMMON_H_
